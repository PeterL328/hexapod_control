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

    // Configure hexapod controller
    HexapodController hexapod_controller{};

    while (ros::ok()) {
        hexapod_controller.state_transition();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
