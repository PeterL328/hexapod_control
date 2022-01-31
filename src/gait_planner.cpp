//
// Created by peter on 2021-07-31.
//

#include <ros/ros.h>
#include <cmath>
#include <Eigen/Geometry>

#include <hexapod_msgs/FeetPositions.h>

#include "gait_planner.h"

using namespace Eigen;

GaitPlanner::GaitPlanner(std::shared_ptr<HexapodModel> model, float publish_rate) : hexapod_model_(model), publish_rate_(publish_rate){
    ros::param::get("GAIT_PERIOD_DISTANCE", gait_period_distance_);
    ros::param::get("LEG_LIFT_HEIGHT", leg_lift_height_);

    // Initialize gait.
    gait_ = std::make_unique<Gait>(Gait::Mode::Tripod);
    gait_seq_ = gait_->get_current_seq_and_next();
    legs_moved_ = std::vector<int>(gait_seq_.size(), 0);
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
    float cycle_distance_meters_local_frame_x = twist.linear.x / publish_rate_;
    float cycle_distance_meters_local_frame_y = twist.linear.y / publish_rate_;

    Vector3f cycle_distance_meters_local_frame(cycle_distance_meters_local_frame_x, cycle_distance_meters_local_frame_y, 0);
    Matrix3f body_rot_mat = hexapod_model_->get_body_rot_mat();

    Vector3f cycle_distance_meters_global_frame = body_rot_mat * cycle_distance_meters_local_frame;

    // Move the body.
    hexapod_model_->move_body_in_body_frame(cycle_distance_meters_global_frame[0] * phase_time_ratio, cycle_distance_meters_global_frame[1] * phase_time_ratio, 0);

    // Get current feet/legs positions
    hexapod_msgs::FeetPositions default_feet_positions_in_body_frame = hexapod_model_->get_initial_feet_positions_in_body_frame();

    hexapod_msgs::FeetPositions current_feet_positions = hexapod_model_->get_feet_positions();

    // Move the legs.
    for (int i = 0; i < 6; i++) {
        float new_x = default_feet_positions_in_body_frame.foot[i].x;
        float new_y = default_feet_positions_in_body_frame.foot[i].y;
        float new_z = default_feet_positions_in_body_frame.foot[i].z;

        if (gait_seq_[i] == 1) {
            new_x += cycle_distance_meters_x * ((-1 * phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + ((1  - phase_time_ratio) * (period_cycle_ + 1)));
            new_y += cycle_distance_meters_y * ((-1 * phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + ((1  - phase_time_ratio) * (period_cycle_ + 1)));

            new_z = leg_lift_height_ * (static_cast<float>(period_cycle_) / period_cycle_length_) - hexapod_model_->get_standing_height();

            // After we lift the leg and move it, mark it.
            legs_moved_[i] = 1;
        }
        else if (legs_moved_[i] == 1) {
            new_x += cycle_distance_meters_x * (((1 - phase_time_ratio * gait_->get_sequence_index()) * (period_cycle_length_)) - (phase_time_ratio * (period_cycle_ + 1)));
            new_y += cycle_distance_meters_y * (((1 - phase_time_ratio * gait_->get_sequence_index()) * (period_cycle_length_)) - (phase_time_ratio * (period_cycle_ + 1)));

            new_z = hexapod_model_->get_standing_height() * -1;
        }
        else {
            new_x -= cycle_distance_meters_x * ((phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + (phase_time_ratio * (period_cycle_ + 1)));
            new_y -= cycle_distance_meters_y * ((phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + (phase_time_ratio * (period_cycle_ + 1)));

            // Since we are working in the local frame, the feet contact point has negative z value.
            new_z = hexapod_model_->get_standing_height() * -1;
        }
        hexapod_model_->set_foot_position_in_body_frame(i, new_x, new_y, new_z);
    }

    period_cycle_++;
    if (period_cycle_ == period_cycle_length_) {
        // Clear the marking vector.
        if (gait_->get_sequence_index() == gait_->get_sequence_size() - 1) {
            std::fill(legs_moved_.begin(), legs_moved_.end(), 0);
        }

        period_cycle_ = 0;
        gait_seq_ = gait_->get_current_seq_and_next();
    }
}
