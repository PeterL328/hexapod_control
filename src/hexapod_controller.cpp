//
// Created by peter on 2021-05-31.
//

#include "hexapod_controller.h"


HexapodController::HexapodController(std::shared_ptr<HexapodModel> hexapod_model_ptr) {
    hexapod_model_ptr_ = hexapod_model_ptr;

    // Configure subscribers.
    twist_command_sub_ = nh_.subscribe(twist_command_topic_name_, 1, &HexapodController::command_message_callback, this);
}

void HexapodController::command_message_callback(geometry_msgs::Twist::ConstPtr twist) {
    twist_.linear.x = twist->linear.x;
    twist_.linear.y = twist->linear.y;
    twist_.angular.z = twist->angular.z;
}