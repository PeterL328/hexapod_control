//
// Created by peter on 2021-05-31.
//

#ifndef HEXAPOD_WS_HEXAPOD_CONTROLLER_H
#define HEXAPOD_WS_HEXAPOD_CONTROLLER_H

#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>
#include <std_msgs/String.h>

#include "hexapod_model.h"
#include "gait_planner.h"
#include "kinematics.h"

class HexapodController {
public:
    /// Creates an instance of a Hexapod Controller object.
    /// \param publish_rate The publish rate of the controller.
    explicit HexapodController(float publish_rate);

    /// Updates the state of the model and executes the appropriate action.
    void state_transition();

private:
    std::shared_ptr<HexapodModel> hexapod_model_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<GaitPlanner> gait_planner_;
    geometry_msgs::Twist twist_;
    geometry_msgs::Twist initial_twist_;
    hexapod_msgs::Pose translate_rotate_pose_;
    hexapod_msgs::Pose initial_translate_rotate_pose_;

    // Used so we can have atomic state transitions for actions spanning multiple "cycles".
    bool state_transitioning_{false};
    float height_adjustment_amount_{0.002f};

    // Loads from parameter server
    float pitch_lower_bound_, pitch_upper_bound_;
    float yaw_lower_bound_, yaw_upper_bound_;
    float roll_lower_bound_, roll_upper_bound_;
    float position_x_lower_bound_, position_x_upper_bound_;
    float position_y_lower_bound_, position_y_upper_bound_;

    // Topic names.
    const std::string joints_command_topic_name_{"joints_command"};
    const std::string translate_rotate_command_topic_name_{"translate_rotate_command"};
    const std::string twist_command_topic_name_{"cmd_vel"};
    const std::string state_command_topic_name_{"state"};

    // Publisher.
    ros::Publisher joints_command_pub_;

    // Subscriber.
    ros::Subscriber twist_command_sub_;
    ros::Subscriber translate_rotate_command_sub_;
    ros::Subscriber state_command_sub_;

    ros::NodeHandle nh_{"~"};

    /// Resets the twist.
    void reset_twist();

    /// Resets the translate rotate.
    void reset_translate_rotate();

    /// Callback function for twist messages.
    /// \param twist The twist geometry message.
    void twist_command_message_callback(geometry_msgs::Twist::ConstPtr twist);

    /// Callback function for twist messages.
    /// \param state The state message.
    void state_command_message_callback(std_msgs::String::ConstPtr state);

    /// Callback function for translate and rotate messages.
    /// \param pose The pose message.
    void translate_rotate_command_message_callback(hexapod_msgs::Pose::ConstPtr pose);

    /// Publishes the joint status to the joints_command topic.
    void publish_joints();

    /// Commands the hexapod to stand up.
    void stand_up();

    /// Commands the hexapod to walk.
    void walk();

    /// Commands the hexapod to sit down.
    void sit_down();

    /// Commands the hexapod to translate and rotate.
    void translate_rotate();
};


#endif //HEXAPOD_WS_HEXAPOD_CONTROLLER_H
