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

hexapod_msgs::Pose HexapodModel::get_body() const {
    return body_;
}

void HexapodModel::set_body_orientation(float pitch, float yaw, float roll) {
    body_.orientation.pitch = pitch;
    body_.orientation.yaw = yaw;
    body_.orientation.roll = roll;
}

void HexapodModel::set_body_position(float x, float y, float z) {
    body_.position.x = x;
    body_.position.y = y;
    body_.position.z = z;
}

void HexapodModel::set_body_z(float z) {
    body_.position.z = z;
}

void HexapodModel::set_body_x(float x) {
    body_.position.x = x;
}

void HexapodModel::set_body_y(float y) {
    body_.position.y = y;
}

hexapod_msgs::FeetPositions HexapodModel::get_feet_positions() const {
    return feet_positions_;
}
