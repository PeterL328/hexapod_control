//
// Created by peter on 2021-05-30.
//

#ifndef HEXAPOD_WS_HEXAPOD_MODEL_H
#define HEXAPOD_WS_HEXAPOD_MODEL_H

#include <Eigen/Dense>

#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Pose.h>

class HexapodModel {
public:
    enum class RobotState {
        Normal,
        TranslateRotate,
        Off
    };

    /// Creates an instance of a Hexapod Model object.
    explicit HexapodModel();

    /// Resets to the default state of the robot.
    /// \return The current robot state.
    void reset();

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

    /// Sets the body pose.
    /// \param pose The new pose.
    void set_body(hexapod_msgs::Pose pose);

    /// Sets the orientation of the body.
    /// \param pitch
    /// \param yaw
    /// \param roll
    void set_body_orientation(float pitch, float yaw, float roll);

    /// Sets the roll of the body.
    /// \param roll
    void set_body_roll(float roll);

    /// Gets the roll of the body.
    /// \return The current orientation roll of the body.
    float get_body_roll() const;

    /// Sets the position of the body.
    /// \param x
    /// \param y
    /// \param z
    void set_body_position(float x, float y, float z);

    /// Sets only the height (z) of the body.
    /// \param z
    void set_body_z(float z);

    /// Gets the height (z) of the body.
    /// \return The current height of the body.
    float get_body_z() const;

    /// Sets only the x of the body.
    /// \param x
    void set_body_x(float x);

    /// Gets the height (x) of the body.
    /// \return The x of the body.
    float get_body_x() const;

    /// Sets only the y of the body.
    /// \param y
    void set_body_y(float y);

    /// Gets the height (y) of the body.
    /// \return The y of the body.
    float get_body_y() const;

    /// Moves the body in the local body frame.
    /// \param x
    /// \param y
    /// \param z
    void move_body_in_body_frame(float x, float y, float z);

    /// Gets the feet positions in global coordinates.
    /// \return The FeetPositions message.
    hexapod_msgs::FeetPositions get_feet_positions() const;

    /// Gets the feet positions in body frame coordinates.
    /// \return The FeetPositions message.
    hexapod_msgs::FeetPositions get_feet_positions_in_body_frame() const;

    /// Gets the initial feet positions in the body frame coordinates
    /// \return The FeetPositions message.
    hexapod_msgs::FeetPositions get_initial_feet_positions_in_body_frame() const;

    /// Sets the position for a foot in global coordinates.
    /// \param leg_index The index of the foot according to ['LF', 'LM', 'LB', 'RF', 'RM', 'RB']
    /// \param x
    /// \param y
    /// \param z
    void set_foot_position(int leg_index, float x, float y, float z);

    /// Sets the position for a foot in body frame coordinates.
    /// This method internally applies transformations to the feet positions which are expressed in the global frame.
    /// \param leg_index The index of the foot according to ['LF', 'LM', 'LB', 'RF', 'RM', 'RB']
    /// \param x
    /// \param y
    /// \param z
    void set_foot_position_in_body_frame(int leg_index, float x, float y, float z);

    /// Gets the vector (x, y, z) from the origin of the body to a particular coxa point (hip joint).
    /// \param leg_index The index of the foot according to ['LF', 'LM', 'LB', 'RF', 'RM', 'RB']
    /// \return The vector (x, y, z).
    Eigen::Vector3f get_center_to_coxa(int leg_index) const;

    /// Gets the length of the coxa.
    /// \return The length of the coxa.
    float get_coxa_length() const;

    /// Gets the length of the femur.
    /// \return The length of the femur.
    float get_femur_length() const;

    /// Gets the length of the tibia.
    /// \return The length of the tibia.
    float get_tibia_length() const;

    /// Gets the height of the body while standing.
    /// \return The height of the body.
    float get_standing_height() const;

    /// Gets the height of the body while sitting.
    /// \return The height of the body.
    float get_sitting_height() const;

    /// Gets the body rotation matrix.
    /// \return The body rotation matrix.
    Eigen::Matrix3f get_body_rot_mat() const;

private:
    RobotState current_status_;
    RobotState previous_status_;

    hexapod_msgs::Pose body_;
    hexapod_msgs::FeetPositions feet_positions_;
    hexapod_msgs::FeetPositions initial_feet_positions_;

    // Loads from parameter server
    std::vector<float> center_to_coxa_x_, center_to_coxa_y_;
    std::vector<float> initial_center_to_feet_x_, initial_center_to_feet_y_, initial_center_to_feet_z_;
    float coxa_length_, femur_length_, tibia_length_;
    float standing_height_, sitting_height_;

    Eigen::Matrix3f euler_angles_to_rotation_matrix(float roll, float yaw, float pitch) const;
};


#endif //HEXAPOD_WS_HEXAPOD_MODEL_H
