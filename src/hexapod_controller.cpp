//
// Created by peter on 2021-05-31.
//

#include <ros/console.h>

#include "hexapod_controller.h"


HexapodController::HexapodController() {
    // Setup the servo controller object
    hexapod_model_ = std::make_unique<HexapodModel>();

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

        // Update state.
        ROS_INFO("Stand up");
        // TODO: Change the state once we know we have completely standing up.
        hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Active);
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Active) {
        // Walking.
        ROS_INFO("Walking");
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Inactive) {
        // Sit down.

        // Update state.
        ROS_INFO("Sit down");
        hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Inactive);
    } else {
        stay_resting();
        ROS_INFO("Stay resting");
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
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Active);
        }
    } else {
        if (current_state == HexapodModel::RobotState::Active) {
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Inactive);
        }
    }
}

void HexapodController::publish_joints() {
    joints_command_pub_.publish(hexapod_model_->get_legs_joints());
}

void HexapodController::stay_resting() {

}