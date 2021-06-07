//
// Created by peter on 2021-05-30.
//

#ifndef HEXAPOD_WS_HEXAPOD_MODEL_H
#define HEXAPOD_WS_HEXAPOD_MODEL_H

#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Pose.h>


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
    /// \param state The current robot state.
    void set_current_robot_status(RobotState state);

    /// Gets the current robot status.
    /// \return The previous robot state.
    RobotState get_previous_robot_status() const;

    /// Sets the current robot status.
    /// \param state The previous robot state.
    void set_previous_robot_status(RobotState state);

    /// Gets the body pose.
    /// \return The body Pose message.
    hexapod_msgs::Pose get_body() const;

    /// Sets the orientation of the body.
    /// \param pitch
    /// \param yaw
    /// \param roll
    void set_body_orientation(float pitch, float yaw, float roll);

    /// Sets the position of the body.
    /// \param x
    /// \param y
    /// \param z
    void set_body_position(float x, float y, float z);

    /// Sets only the height (z) of the body.
    /// \param z
    void set_body_z(float z);

    /// Sets only the x of the body.
    /// \param x
    void set_body_x(float x);

    /// Sets only the y of the body.
    /// \param y
    void set_body_y(float y);

    /// Gets the feet positions.
    /// \return The FeetPositions message.
    hexapod_msgs::FeetPositions get_feet_positions() const;

private:
    RobotState current_status_;
    RobotState previous_status_;

    hexapod_msgs::Pose body_;
    hexapod_msgs::FeetPositions feet_positions_;
};


#endif //HEXAPOD_WS_HEXAPOD_MODEL_H