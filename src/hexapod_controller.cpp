//
// Created by peter on 2021-05-31.
//

#include <ros/console.h>
#include <cmath>

#include "hexapod_controller.h"

HexapodController::HexapodController() {
    // Setup the hexapod model object
    hexapod_model_ = std::make_shared<HexapodModel>();
    kinematics_ = std::make_unique<Kinematics>(hexapod_model_);

    // Configure publisher.
    joints_command_pub_ = nh_.advertise<hexapod_msgs::LegsJoints>(joints_command_topic_name_, 1);

    // Configure subscribers.
    twist_command_sub_ = nh_.subscribe(twist_command_topic_name_, 1, &HexapodController::command_message_callback, this);
    state_command_sub_ = nh_.subscribe(state_command_topic_name_, 1, &HexapodController::state_message_callback, this);
}

void HexapodController::state_transition() {
    HexapodModel::RobotState current_state = hexapod_model_->get_current_robot_status();
    HexapodModel::RobotState previous_state = hexapod_model_->get_previous_robot_status();
    if (previous_state == HexapodModel::RobotState::Inactive && current_state == HexapodModel::RobotState::Active) {
        // Stand up.
        stand_up();
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Active) {
        // Walking.
        walk();
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Inactive) {
        // Sit down.
        sit_down();
    } else {
        stay_resting();
    }
    publish_joints();
}

void HexapodController::command_message_callback(geometry_msgs::Twist::ConstPtr twist) {
    twist_.linear.x = twist->linear.x;
    twist_.linear.y = twist->linear.y;
    twist_.angular.z = twist->angular.z;
}

void HexapodController::state_message_callback(std_msgs::Bool::ConstPtr state) {
    HexapodModel::RobotState current_state = hexapod_model_->get_current_robot_status();
    if (state->data == true) {
        if (current_state == HexapodModel::RobotState::Inactive) {
            hexapod_model_->set_body_orientation(0, 0, 0);
            hexapod_model_->set_body_position(0, 0 ,0);
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Active);
        }
    } else {
        if (current_state == HexapodModel::RobotState::Active) {
            hexapod_model_->set_body_orientation(0, 0, 0);
            hexapod_model_->set_body_x(0);
            hexapod_model_->set_body_y(0);
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Inactive);
        }
    }
}

void HexapodController::publish_joints() {
    hexapod_msgs::LegsJoints legs_joints = kinematics_->body_feet_config_to_legs_joints(
            hexapod_model_->get_body(),
            hexapod_model_->get_feet_positions());
    joints_command_pub_.publish(legs_joints);
}

void HexapodController::stand_up() {
    ROS_INFO("Stand up");
    float current_height = hexapod_model_->get_body_z();
    float target_height = hexapod_model_->get_standing_height();
    if (current_height < target_height) {
        hexapod_model_->set_body_z(std::min(target_height, current_height + 0.01f));
        if (std::abs(hexapod_model_->get_body_z() - target_height) <= 0.001f) {
            hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Active);
        }
    }
}

void HexapodController::walk() {
    ROS_INFO("Walking");
}

void HexapodController::sit_down() {
    ROS_INFO("Sit down");
    float current_height = hexapod_model_->get_body_z();
    float target_height = hexapod_model_->get_standing_height();
    if (current_height > 0.f) {
        hexapod_model_->set_body_z(std::max(0.f, current_height - 0.01f));
        if (std::abs(hexapod_model_->get_body_z() - target_height) <= 0.001f) {
            hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Inactive);
        }
    }
}

void HexapodController::stay_resting() {
    ROS_INFO("Stay resting");
}