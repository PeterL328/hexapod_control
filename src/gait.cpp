//
// Created by peter on 2021-07-31.
//

#include <ros/ros.h>
#include <cmath>

#include "gait.h"

Gait::Gait(std::shared_ptr<HexapodModel> model, float publish_rate) : hexapod_model_(model), publish_rate_(publish_rate), gait_mode_(Mode::Tripod){
    ros::param::get("LEG_WALK_DISTANCE", leg_walk_distance_);
}

void Gait::UpdateGaitMode(Mode new_mode) {
    gait_mode_ = new_mode;
}

void Gait::UpdateModel(geometry_msgs::Twist& twist) {
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

    // Move the legs.
    // TODO: Fill in

    period_cycle_++;
}
