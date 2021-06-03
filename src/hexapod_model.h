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

    /// Gets the current robot status.
    /// \return The current robot state.
    RobotState get_current_robot_status() const;

    /// Sets the current robot status.
    /// \param state
    void set_current_robot_status(RobotState state);

    /// Gets the current robot status.
    /// \return The previous robot state.
    RobotState get_previous_robot_status() const;

    /// Sets the current robot status.
    /// \param The previous robot state.
    void set_previous_robot_status(RobotState state);

    /// Gets the legs joints.
    /// \return The legs joints message.
    hexapod_msgs::LegsJoints get_legs_joints() const;

private:
    RobotState current_status_;
    RobotState previous_status_;

    hexapod_msgs::LegsJoints legs_joints_;
};


#endif //HEXAPOD_WS_HEXAPOD_MODEL_H
