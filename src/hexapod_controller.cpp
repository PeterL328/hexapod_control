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
    gait_ = std::make_unique<Gait>(hexapod_model_);

    // Load from parameter server.
    ros::param::get("PITCH_LOWER_BOUND", pitch_lower_bound_);
    ros::param::get("PITCH_UPPER_BOUND", pitch_upper_bound_);

    ros::param::get("YAW_LOWER_BOUND", yaw_lower_bound_);
    ros::param::get("YAW_UPPER_BOUND", yaw_upper_bound_);

    ros::param::get("ROLL_LOWER_BOUND", roll_lower_bound_);
    ros::param::get("ROLL_UPPER_BOUND", roll_upper_bound_);

    ros::param::get("POSITION_X_LOWER_BOUND", position_x_lower_bound_);
    ros::param::get("POSITION_X_UPPER_BOUND", position_x_upper_bound_);

    ros::param::get("POSITION_Y_LOWER_BOUND", position_y_lower_bound_);
    ros::param::get("POSITION_Y_UPPER_BOUND", position_y_upper_bound_);

    // Set initial state
    initial_twist_.linear.x = 0.f;
    initial_twist_.linear.y = 0.f;
    initial_twist_.linear.z = 0.f;
    initial_twist_.angular.x = 0.f;
    initial_twist_.angular.y = 0.f;
    initial_twist_.angular.z = 0.f;

    initial_translate_rotate_pose_.position.x = 0.f;
    initial_translate_rotate_pose_.position.y = 0.f;
    initial_translate_rotate_pose_.position.z = 0.f;
    initial_translate_rotate_pose_.orientation.roll = 0.f;
    initial_translate_rotate_pose_.orientation.yaw = 0.f;
    initial_translate_rotate_pose_.orientation.pitch = 0.f;

    reset_twist();
    reset_translate_rotate();

    // Configure publisher.
    joints_command_pub_ = nh_.advertise<hexapod_msgs::LegsJoints>(joints_command_topic_name_, 1);

    // Configure subscribers.
    twist_command_sub_ = nh_.subscribe(twist_command_topic_name_, 1, &HexapodController::twist_command_message_callback, this);
    state_command_sub_ = nh_.subscribe(state_command_topic_name_, 1, &HexapodController::state_command_message_callback, this);
    translate_rotate_command_sub_ = nh_.subscribe(translate_rotate_command_topic_name_, 1, &HexapodController::translate_rotate_command_message_callback, this);
}

void HexapodController::state_transition() {
    HexapodModel::RobotState current_state = hexapod_model_->get_current_robot_status();
    HexapodModel::RobotState previous_state = hexapod_model_->get_previous_robot_status();
    if (previous_state == HexapodModel::RobotState::Off && current_state == HexapodModel::RobotState::Normal) {
        // Stand up.
        stand_up();
    }
    else if (previous_state == HexapodModel::RobotState::Normal && current_state == HexapodModel::RobotState::Normal) {
        // Walking.
        walk();
    }
    else if (previous_state == HexapodModel::RobotState::TranslateRotate && current_state == HexapodModel::RobotState::TranslateRotate) {
        translate_rotate();
    }
    else if ((previous_state == HexapodModel::RobotState::Normal || previous_state == HexapodModel::RobotState::TranslateRotate) && current_state == HexapodModel::RobotState::Off) {
        // Sit down.
        sit_down();

        // Reset
        reset_twist();
        reset_translate_rotate();
    }
    else if (previous_state == HexapodModel::RobotState::Normal && current_state == HexapodModel::RobotState::TranslateRotate) {
        hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::TranslateRotate);
        reset_twist();
    }
    else if (previous_state == HexapodModel::RobotState::TranslateRotate && current_state == HexapodModel::RobotState::Normal) {
        hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Normal);
        reset_translate_rotate();
    }

    publish_joints();
}

void HexapodController::reset_twist() {
    twist_ = initial_twist_;
}

void HexapodController::reset_translate_rotate() {
    translate_rotate_pose_ = initial_translate_rotate_pose_;
}

void HexapodController::twist_command_message_callback(geometry_msgs::Twist::ConstPtr twist) {
    twist_.linear.x = twist->linear.x;
    twist_.linear.y = twist->linear.y;
    twist_.angular.z = twist->angular.z;
}

void HexapodController::state_command_message_callback(std_msgs::String::ConstPtr state) {
    // If in progress of a state transition then skip.
    if (state_transitioning_) {
        ROS_INFO("State still transitioning so ignoring new state.");
        return;
    }
    HexapodModel::RobotState current_state = hexapod_model_->get_current_robot_status();
    hexapod_model_->set_body_orientation(0, 0, 0);
    hexapod_model_->set_body_x(0);
    hexapod_model_->set_body_y(0);
    if (state->data == "Normal") {
        if (current_state == HexapodModel::RobotState::Off) {
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Normal);
            state_transitioning_ = true;
        }
        else if (current_state == HexapodModel::RobotState::TranslateRotate) {
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Normal);
        }
    } else if (state->data == "Off") {
        hexapod_model_->set_current_robot_status(HexapodModel::RobotState::Off);
        state_transitioning_ = true;
    } else if (state->data == "TranslateRotate") {
        if (current_state == HexapodModel::RobotState::Normal) {
            hexapod_model_->set_current_robot_status(HexapodModel::RobotState::TranslateRotate);
        }
    } else {
        ROS_ERROR("%s is not a valid state.", state->data.c_str());
    }
}

void HexapodController::translate_rotate_command_message_callback(hexapod_msgs::Pose::ConstPtr pose) {
    translate_rotate_pose_.position.x = pose->position.x;
    translate_rotate_pose_.position.y = pose->position.y;
    translate_rotate_pose_.position.z = 0.f;

    translate_rotate_pose_.orientation.roll = pose->orientation.roll;
    translate_rotate_pose_.orientation.yaw = pose->orientation.yaw;
    translate_rotate_pose_.orientation.pitch = pose->orientation.pitch;
}

void HexapodController::publish_joints() {
    hexapod_msgs::LegsJoints legs_joints = kinematics_->body_feet_config_to_legs_joints();
    joints_command_pub_.publish(legs_joints);
}

void HexapodController::stand_up() {
    ROS_INFO("Stand up");
    float current_height = hexapod_model_->get_body_z();
    float target_standing_height = hexapod_model_->get_standing_height();

    float height_increment_amount = height_adjustment_amount_;
    float error_allow_bound = 0.0001f;

    if (current_height < target_standing_height) {
        hexapod_model_->set_body_z(std::min(target_standing_height, current_height + height_increment_amount));
        if (std::abs(hexapod_model_->get_body_z() - target_standing_height) <= error_allow_bound) {
            hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Normal);
            state_transitioning_ = false;
        }
    }
}

void HexapodController::walk() {
    gait_->UpdateModel(twist_);
}

void HexapodController::sit_down() {
    ROS_INFO("Sit down");
    float current_height = hexapod_model_->get_body_z();
    float target_sitting_height = hexapod_model_->get_sitting_height();

    float height_decrement_amount = height_adjustment_amount_;
    float error_allow_bound = 0.0001f;

    if (current_height > target_sitting_height) {
        hexapod_model_->set_body_z(std::max(target_sitting_height, current_height - height_decrement_amount));
        if (std::abs(hexapod_model_->get_body_z() - target_sitting_height) <= error_allow_bound) {
            hexapod_model_->set_previous_robot_status(HexapodModel::RobotState::Off);
            state_transitioning_ = false;
        }
    }
}

void HexapodController::translate_rotate() {
    // Crop the parameters to be between the bounds.
    float pitch = std::max(pitch_lower_bound_, std::min(pitch_upper_bound_, static_cast<float>(translate_rotate_pose_.orientation.pitch)));
    float yaw = std::max(yaw_lower_bound_, std::min(yaw_upper_bound_, static_cast<float>(translate_rotate_pose_.orientation.yaw)));
    float roll = std::max(roll_lower_bound_, std::min(roll_upper_bound_, static_cast<float>(translate_rotate_pose_.orientation.roll)));
    float position_x = std::max(position_x_lower_bound_, std::min(position_x_upper_bound_, static_cast<float>(translate_rotate_pose_.position.x)));
    float position_y = std::max(position_y_lower_bound_, std::min(position_y_upper_bound_, static_cast<float>(translate_rotate_pose_.position.y)));

    hexapod_model_->set_body_orientation(
        pitch,
        yaw,
        roll);
    hexapod_model_->set_body_position(
        position_x,
        position_y,
        hexapod_model_->get_standing_height());
}
