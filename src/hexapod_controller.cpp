//
// Created by peter on 2021-05-31.
//

#include "hexapod_controller.h"


HexapodController::HexapodController() {
    // Configure publisher.
    joints_command_pub_ = nh_.advertise<hexapod_msgs::LegsJoints>(joints_command_topic_name_, 1);

    // Configure subscribers.
    twist_command_sub_ = nh_.subscribe(twist_command_topic_name_, 1, &HexapodController::command_message_callback, this);
}

void HexapodController::state_transition() {
    HexapodModel::RobotState current_state = hexapod_model_->get_current_robot_status();
    HexapodModel::RobotState previous_state = hexapod_model_->get_previous_robot_status();
    if (previous_state == HexapodModel::RobotState::Inactive && current_state == HexapodModel::RobotState::Active) {
        // Stand up.
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Active) {
        // Walking.
    }
    else if (previous_state == HexapodModel::RobotState::Active && current_state == HexapodModel::RobotState::Inactive) {
        // Sit down.
    } else {
        // Stay down.
    }
    publish_joints();
}


void HexapodController::command_message_callback(geometry_msgs::Twist::ConstPtr twist) {
    twist_.linear.x = twist->linear.x;
    twist_.linear.y = twist->linear.y;
    twist_.angular.z = twist->angular.z;
}

void HexapodController::publish_joints() {
    joints_command_pub_.publish(hexapod_model_->get_legs_joints());
}