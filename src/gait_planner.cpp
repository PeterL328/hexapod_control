//
// Created by peter on 2021-07-31.
//

#include <ros/ros.h>
#include <cmath>

#include <hexapod_msgs/FeetPositions.h>

#include "gait_planner.h"

GaitPlanner::GaitPlanner(std::shared_ptr<HexapodModel> model, float publish_rate) : hexapod_model_(model), publish_rate_(publish_rate){
    ros::param::get("GAIT_PERIOD_DISTANCE", gait_period_distance_);
    ros::param::get("LEG_LIFT_HEIGHT", leg_lift_height_);

    // Initialize gait.
    gait_ = std::make_unique<Gait>(Gait::Mode::Tripod);
    gait_seq_ = gait_->get_current_seq_and_next();
}

void GaitPlanner::update_gait_mode(Gait::Mode new_mode) {
    gait_->update_gait_mode(new_mode);
}

void GaitPlanner::update_model(geometry_msgs::Twist& twist) {
    // TODO: If we are in the middle of a leg movement and get a 0 linear twist then we should reset all legs.
    // Calculate the magnitude of the twist linear speed.
    float linear_speed_magnitude = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
    if (linear_speed_magnitude <= 0.001f) {
        return;
    }

    // Most likely the speed changed, so the period length also changed.
    previous_period_cycle_length_ = period_cycle_length_;

    // Each gait period has many cycles depending on the gait mode. We can calculate the distance to be traversed
    // in one cycle.
    float phase_time_ratio = 1 - gait_->get_duty_factor();
    float distance_per_cycle = gait_period_distance_ * phase_time_ratio;

    // (speed / publish rate) is the distance to be traversed in one compute frame (cycle).
    // This cycle is not the same as the gait cycle.
    period_cycle_length_ = distance_per_cycle / (linear_speed_magnitude / publish_rate_);

    if (previous_period_cycle_length_ != 0) {
        period_cycle_ = round((static_cast<float>(period_cycle_) / previous_period_cycle_length_) * period_cycle_length_);
    }

    // Get the distances in x and y-axis to move for one cycle (frame)
    float cycle_distance_meters_x = twist.linear.x / publish_rate_;
    float cycle_distance_meters_y = twist.linear.y / publish_rate_;

    // Move the body.
    float new_body_position_x = hexapod_model_->get_body_x() + cycle_distance_meters_x * phase_time_ratio;
    float new_body_position_y = hexapod_model_->get_body_y() + cycle_distance_meters_y * phase_time_ratio;

    hexapod_model_->set_body_x(new_body_position_x);
    hexapod_model_->set_body_y(new_body_position_y);

    // Get current feet/legs positions
    hexapod_msgs::FeetPositions current_feet_positions = hexapod_model_->get_feet_positions();

    // Move the legs.
    for (int i = 0; i < 6; i++) {
        float new_x = current_feet_positions.foot[i].x;
        float new_y = current_feet_positions.foot[i].y;
        float new_z = current_feet_positions.foot[i].z;

        if (gait_seq_[i] == 1) {
            new_x += cycle_distance_meters_x;
            new_y += cycle_distance_meters_y;
            new_z = leg_lift_height_ * (static_cast<float>(period_cycle_) / period_cycle_length_);
        } else {
            new_z = 0;
        }
        hexapod_model_->set_foot_position(i, new_x, new_y, new_z);
    }

    period_cycle_++;
    if (period_cycle_ == period_cycle_length_) {
        period_cycle_ = 0;
        gait_seq_ = gait_->get_current_seq_and_next();
    }
}
