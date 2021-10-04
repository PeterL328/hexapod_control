//
// Created by peter on 2021-07-31.
//

#include <ros/ros.h>
#include <cmath>

#include <hexapod_msgs/FeetPositions.h>

#include "gait.h"

Gait::Gait(std::shared_ptr<HexapodModel> model, float publish_rate) : hexapod_model_(model), publish_rate_(publish_rate){
    ros::param::get("LEG_WALK_DISTANCE", leg_walk_distance_);
    ros::param::get("LEG_LIFT_HEIGHT", leg_lift_height_);
    update_gait_mode(Mode::Tripod);
}

void Gait::update_gait_mode(Mode new_mode) {
    gait_mode_ = new_mode;
    if (new_mode == Mode::Tripod) {
        gait_seq_ = {1, 0, 1, 0, 1, 0};
    }
}

void Gait::update_model(geometry_msgs::Twist& twist) {
    // TODO: If we are in the middle of a leg movement and get a 0 linear twist then we should reset all legs.
    // Calculate the magnitude of the twist linear speed.
    float linear_speed_magnitude = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));

    // Update the number of cycles or frames need to complete one period.
    // Most likely the speed changed, so the period length also changed.
    previous_period_cycle_length_ = period_cycle_length_;
    period_cycle_length_ = leg_walk_distance_ / (linear_speed_magnitude / publish_rate_);
    // TODO: Should this be rounded like this?
    period_cycle_ = (static_cast<float>(period_cycle_) / previous_period_cycle_length_) * period_cycle_length_;

    // Get the distances in x and y axis to move for one cycle (frame)
    float cycle_distance_meters_x = twist.linear.x / publish_rate_;
    float cycle_distance_meters_y = twist.linear.y / publish_rate_;

    // Move the body.
    hexapod_model_->set_body_x(hexapod_model_->get_body_x() + cycle_distance_meters_x);
    hexapod_model_->set_body_y(hexapod_model_->get_body_y() + cycle_distance_meters_y);

    // Get current feet/legs positions
    hexapod_msgs::FeetPositions current_feet_positions = hexapod_model_->get_feet_positions();

    // Move the legs.
    for (int i = 0; i < 6; i++) {
        float new_x = 0.f;
        float new_y = 0.f;
        float new_z = 0.f;
        if (gait_seq_[i] == 1) {
            new_x = current_feet_positions.foot[i].x + cycle_distance_meters_x;
            new_y = current_feet_positions.foot[i].y + cycle_distance_meters_y;
            new_z = leg_lift_height_ * (static_cast<float>(period_cycle_) / period_cycle_length_);
        } else {
            new_x = current_feet_positions.foot[i].x - cycle_distance_meters_x;
            new_y = current_feet_positions.foot[i].y - cycle_distance_meters_y;
            new_z = 0;
        }
        hexapod_model_->set_foot_position(i, new_x, new_y, new_z);
    }

    period_cycle_++;
    if (period_cycle_ == period_cycle_length_) {
        period_cycle_ = 0;
        update_gait_seq();
    }
}

void Gait::update_gait_seq() {
    if (gait_mode_ == Mode::Tripod) {
        for (int i = 0; i < gait_seq_.size(); i++) {
            gait_seq_[i] = 1 - gait_seq_[i];
        }
    }
}
