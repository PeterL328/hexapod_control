//
// Created by peter on 2021-05-30.
//
#include <memory>

#include <ros/ros.h>
#include <ros/console.h>

#include "hexapod_controller.h"
#include "hexapod_model.h"

int main(int argc, char **argv)
{
    const std::string node_name = "hexapod_control";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("~");

    // Configure publish loop rate.
    float publish_rate_in_hz;
    nh.param("publish_rate", publish_rate_in_hz, 50.0f);
    ROS_INFO("Publishing at %fHz on %s.", publish_rate_in_hz, node_name.c_str());
    ros::Rate loop_rate(publish_rate_in_hz);

    // Configure hexapod model and controller
    std::shared_ptr<HexapodModel> hexapod_model_ptr = std::make_shared<HexapodModel>();
    HexapodController hexapod_controller{hexapod_model_ptr};

    // Rough steps
    // 2) Ros loop
    while (ros::ok()) {
        HexapodModel::RobotState current_state = hexapod_model_ptr->get_current_robot_status();
        HexapodModel::RobotState previous_state = hexapod_model_ptr->get_previous_robot_status();

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
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
