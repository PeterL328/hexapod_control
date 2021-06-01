//
// Created by peter on 2021-05-30.
//

#ifndef HEXAPOD_WS_HEXAPOD_MODEL_H
#define HEXAPOD_WS_HEXAPOD_MODEL_H

#include <ros/ros.h>
#include <hexapod_msgs/LegsJoints.h>

class HexapodModel {
public:
    enum class RobotState {
        Active,
        Inactive
    };

    /// Creates an instance of a Hexapod Model object.
    explicit HexapodModel();

    /// Publishes the joint status to the joints_command topic.
    void publish_joints();

    /// Gets the current robot status.
    RobotState get_current_robot_status();

    /// Gets the current robot status.
    RobotState get_previous_robot_status();

private:
    RobotState current_status_;
    RobotState previous_status_;

    hexapod_msgs::LegsJoints legs_joints_;

    // Topic names.
    const std::string joints_command_topic_name_{"joints_command"};

    // Publisher.
    ros::Publisher joints_command_pub_;

    ros::NodeHandle nh_{"~"};
};


#endif //HEXAPOD_WS_HEXAPOD_MODEL_H
