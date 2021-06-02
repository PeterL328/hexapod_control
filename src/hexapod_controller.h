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
    explicit HexapodController();


    /// Updates the state of the model and executes the appropriate action.
    void state_transition();

    /// Callback function for twist messages.
    /// \param twist The twist geometry message.
    void command_message_callback(geometry_msgs::Twist::ConstPtr twist);

private:
    std::unique_ptr<HexapodModel> hexapod_model_;
    geometry_msgs::Twist twist_;

    // Topic names.
    const std::string twist_command_topic_name_{"cmd_vel"};
    const std::string joints_command_topic_name_{"joints_command"};

    // Publisher.
    ros::Publisher joints_command_pub_;

    // Subscriber.
    ros::Subscriber twist_command_sub_;

    ros::NodeHandle nh_{"~"};

    /// Publishes the joint status to the joints_command topic.
    void publish_joints();
};


#endif //HEXAPOD_WS_HEXAPOD_CONTROLLER_H
