//
// Created by peter on 2021-05-31.
//

#ifndef HEXAPOD_WS_HEXAPOD_CONTROLLER_H
#define HEXAPOD_WS_HEXAPOD_CONTROLLER_H

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hexapod_msgs/LegsJoints.h>
#include <std_msgs/Bool.h>

#include "hexapod_model.h"
#include "kinematics.h"


class HexapodController {
public:
    /// Creates an instance of a Hexapod Controller object.
    explicit HexapodController();

    /// Updates the state of the model and executes the appropriate action.
    void state_transition();

private:
    std::shared_ptr<HexapodModel> hexapod_model_;
    std::unique_ptr<Kinematics> kinematics_;
    geometry_msgs::Twist twist_;

    // Topic names.
    const std::string joints_command_topic_name_{"joints_command"};
    const std::string twist_command_topic_name_{"cmd_vel"};
    const std::string state_command_topic_name_{"state"};

    // Publisher.
    ros::Publisher joints_command_pub_;

    // Subscriber.
    ros::Subscriber twist_command_sub_;
    ros::Subscriber state_command_sub_;

    ros::NodeHandle nh_{"~"};

    /// Callback function for twist messages.
    /// \param twist The twist geometry message.
    void command_message_callback(geometry_msgs::Twist::ConstPtr twist);

    /// Callback function for twist messages.
    /// \param twist The twist geometry message.
    void state_message_callback(std_msgs::Bool::ConstPtr state);

    /// Publishes the joint status to the joints_command topic.
    void publish_joints();

    /// Start the robot in its initialize position and orientation.
    void initial_configuration();

    /// Commands the hexapod to stand up.
    void stand_up();

    /// Commands the hexapod to walk.
    void walk();

    /// Commands the hexapod to sit down.
    void sit_down();

    /// Commands the hexapod to stay in the resting position.
    void stay_resting();
};


#endif //HEXAPOD_WS_HEXAPOD_CONTROLLER_H
