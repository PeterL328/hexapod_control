//
// Created by peter on 2021-06-07.
//

#ifndef HEXAPOD_WS_KINEMATICS_H
#define HEXAPOD_WS_KINEMATICS_H

#include <Eigen/Dense>

#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>

#include "hexapod_model.h"

class Kinematics {
public:
    /// Creates an instance of a Kinematics object.
    /// \model body A shared pointer to the hexapod model.
    explicit Kinematics(std::shared_ptr<HexapodModel> model);

    /// Calculates the legs joints angles given body & feet position and orientations.
    /// \return The legs joints message.
    hexapod_msgs::LegsJoints body_feet_config_to_legs_joints();

private:
    std::shared_ptr<HexapodModel> hexapod_model_;

    Eigen::Matrix3f euler_angles_to_rotation_matrix(float roll, float yaw, float pitch);
};


#endif //HEXAPOD_WS_KINEMATICS_H
