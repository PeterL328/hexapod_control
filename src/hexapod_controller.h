//
// Created by peter on 2021-05-31.
//

#ifndef HEXAPOD_WS_HEXAPOD_CONTROLLER_H
#define HEXAPOD_WS_HEXAPOD_CONTROLLER_H

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "hexapod_model.h"


class HexapodController {
public:
    /// Creates an instance of a Hexapod Controller object.
    explicit HexapodController(std::shared_ptr<HexapodModel> hexapod_model_ptr);

    /// Callback function for twist messages.
    void command_message_callback(geometry_msgs::Twist::ConstPtr twist);
private:
    std::shared_ptr<HexapodModel> hexapod_model_ptr_;
    geometry_msgs::Twist twist_;

    // Topic name.
    const std::string twist_command_topic_name_{"cmd_vel"};

    // Subscriber.
    ros::Subscriber twist_command_sub_;

    ros::NodeHandle nh_{"~"};
};


#endif //HEXAPOD_WS_HEXAPOD_CONTROLLER_H
