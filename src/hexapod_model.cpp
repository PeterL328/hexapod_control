//
// Created by peter on 2021-05-30.
//

#include "hexapod_model.h"

HexapodModel::HexapodModel() {
    // Read from parameter server to populate values.
    current_status_ = HexapodModel::RobotState::Inactive;
    previous_status_ = HexapodModel::RobotState::Inactive;
}

HexapodModel::RobotState HexapodModel::get_current_robot_status() const {
    return current_status_;
}

void HexapodModel::set_current_robot_status(RobotState state) {
    current_status_ = state;
}

HexapodModel::RobotState HexapodModel::get_previous_robot_status() const {
    return previous_status_;
}

void HexapodModel::set_previous_robot_status(RobotState state) {
    previous_status_ = state;
}

hexapod_msgs::LegsJoints HexapodModel::get_legs_joints() const {
    return legs_joints_;
}