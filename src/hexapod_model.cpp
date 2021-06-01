//
// Created by peter on 2021-05-30.
//

#include "hexapod_model.h"

HexapodModel::HexapodModel() {
    // Read from parameter server to populate values.
    current_status_ = HexapodModel::RobotState::Inactive;
    previous_status_ = HexapodModel::RobotState::Inactive;

    // Configure publisher.
    joints_command_pub_ = nh_.advertise<hexapod_msgs::LegsJoints>(joints_command_topic_name_, 1);
}

void HexapodModel::publish_joints() {
    joints_command_pub_.publish(legs_joints_);
}

HexapodModel::RobotState HexapodModel::get_current_robot_status() {
    return current_status_;
}

HexapodModel::RobotState HexapodModel::get_previous_robot_status() {
    return previous_status_;
}