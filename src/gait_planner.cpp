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
    ros::param::get("GAIT_PERIOD_ROTATION", gait_period_rotation_);
    ros::param::get("LEG_LIFT_HEIGHT", leg_lift_height_);

    // Initialize gait.
    gait_ = std::make_unique<Gait>(Gait::Mode::Tripod);
    reset_state();
}

void GaitPlanner::update_gait_mode(Gait::Mode new_mode) {
    gait_->update_gait_mode(new_mode);
}

void GaitPlanner::reset_state() {
    // Reset gait
    gait_->reset_sequence();
    gait_seq_ = gait_->get_current_seq_and_next();
    legs_moved_ = std::vector<int>(gait_seq_.size(), 0);

    // Reset period
    period_cycle_ = 0;
    was_travelling_ = false;
    is_travelling_ = false;
    force_extra_period_ = false;
}

void GaitPlanner::update_model(geometry_msgs::Twist& twist) {
    // Calculate the magnitude of the twist linear speed.
    float linear_speed_magnitude = sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2));
    float angular_speed_magnitude = abs(twist.angular.x);
    float angular_distance_per_rate = angular_speed_magnitude / publish_rate_;

    float phase_time_ratio = 1 - gait_->get_duty_factor();
    float cycle_distance_meters_global_frame_x = 0.f;
    float cycle_distance_meters_global_frame_y = 0.f;

    if (linear_speed_magnitude >= linear_deadzone_ || angular_speed_magnitude >= angular_deadzone_) {
        is_travelling_ = true;
        if (!was_travelling_) {
            period_cycle_ = 0;
        }
        // Most likely the speed changed, so the period length also changed.
        previous_period_cycle_length_ = period_cycle_length_;

        // Each gait period has many cycles depending on the gait mode. We can calculate the distance to be traversed
        // in one cycle.
        // TODO: Introducing rotation, distance to be traversed in one cycle may be different for each leg
        float distance_per_cycle = (gait_period_distance_) * phase_time_ratio;

        // (speed / publish rate) is the distance to be traversed in one compute frame (cycle).
        // This cycle is not the same as the gait cycle.
        // TODO: Speed magnitude needs to include the angular speed
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

        cycle_distance_meters_global_frame_x = cycle_distance_meters_global_frame[0];
        cycle_distance_meters_global_frame_y = cycle_distance_meters_global_frame[1];
    } else {
        is_travelling_ = false;
        // Force extra cycle here to get period of 0 so legs are touching the ground.
        if (was_travelling_ || force_extra_period_) {
            previous_period_cycle_length_ = period_cycle_length_;
            period_cycle_length_ = reset_leg_period_cycle_length_;

            if (previous_period_cycle_length_ != 0) {
                period_cycle_ = round((static_cast<float>(period_cycle_) / previous_period_cycle_length_) * period_cycle_length_);
            }
            force_extra_period_ = false;
        } else {
            return;
        }
    }

    // Move the body.
    hexapod_model_->move_body_in_body_frame(cycle_distance_meters_global_frame_x * phase_time_ratio, cycle_distance_meters_global_frame_y * phase_time_ratio, 0);

    // Get default feet/legs positions
    hexapod_msgs::FeetPositions default_feet_positions_in_body_frame = hexapod_model_->get_initial_feet_positions_in_body_frame();

    // Get current feet/legs positions in body frame
    hexapod_msgs::FeetPositions feet_positions_in_body_frame = hexapod_model_->get_feet_positions_in_body_frame();

    // Move the legs.
    for (int i = 0; i < 6; i++) {
        float new_x = default_feet_positions_in_body_frame.foot[i].x;
        float new_y = default_feet_positions_in_body_frame.foot[i].y;
        float new_z = default_feet_positions_in_body_frame.foot[i].z;

        if (angular_speed_magnitude >= angular_deadzone_) {
            // To add rotation, find vector perpendicular to the center_to_feet vector.
            // There will be two perpendicular, choose the one that matches the direction.
            Vector2f center_to_feet(
                feet_positions_in_body_frame.foot[i].x,
                feet_positions_in_body_frame.foot[i].y);

            Vector2f perpendicular(0.f, 0.f);
            if (twist.angular.x > 0.f) {
                perpendicular = get_perpendicular_clockwise(center_to_feet);
            } else {
                perpendicular = get_perpendicular_counterclockwise(center_to_feet);
            }

            // Scale the perpendicular vector by how much we want to rotate.
            // Find the arc length based on the angular velocity and radius (which is center to feet).
            perpendicular *= angular_distance_per_rate * center_to_feet.norm();

            cycle_distance_meters_global_frame_x += perpendicular[0];
            cycle_distance_meters_global_frame_y += perpendicular[1];
        }

        if (gait_seq_[i] == 1) {
            new_x += cycle_distance_meters_global_frame_x * ((-1 * phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + ((1  - phase_time_ratio) * (period_cycle_ + 1)));
            new_y += cycle_distance_meters_global_frame_y * ((-1 * phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + ((1  - phase_time_ratio) * (period_cycle_ + 1)));

            new_z = leg_lift_height_ * (static_cast<float>(period_cycle_) / period_cycle_length_) - hexapod_model_->get_standing_height();

            // After we lift the leg and move it, mark it.
            legs_moved_[i] = 1;
        }
        else if (legs_moved_[i] == 1) {
            new_x += cycle_distance_meters_global_frame_x * (((1 - phase_time_ratio * gait_->get_sequence_index()) * (period_cycle_length_)) - (phase_time_ratio * (period_cycle_ + 1)));
            new_y += cycle_distance_meters_global_frame_y * (((1 - phase_time_ratio * gait_->get_sequence_index()) * (period_cycle_length_)) - (phase_time_ratio * (period_cycle_ + 1)));

            new_z = hexapod_model_->get_standing_height() * -1;
        }
        else {
            new_x -= cycle_distance_meters_global_frame_x * ((phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + (phase_time_ratio * (period_cycle_ + 1)));
            new_y -= cycle_distance_meters_global_frame_y * ((phase_time_ratio * gait_->get_sequence_index() * (period_cycle_length_)) + (phase_time_ratio * (period_cycle_ + 1)));

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

        if (was_travelling_ && !is_travelling_) {
            was_travelling_ = false;
            force_extra_period_ = true;
        }
    }

    if (is_travelling_) {
        was_travelling_ = true;
    }
}

Vector2f GaitPlanner::get_perpendicular_clockwise(const Vector2f &vec) const {
    return Vector2f(vec[1], -vec[0]).normalized();
}

Vector2f GaitPlanner::get_perpendicular_counterclockwise(const Vector2f &vec) const {
    return Vector2f(-vec[1], vec[0]).normalized();
}
