//
// Created by peter on 2021-06-07.
//

#ifndef HEXAPOD_WS_KINEMATICS_H
#define HEXAPOD_WS_KINEMATICS_H

#include <hexapod_msgs/FeetPositions.h>
#include <hexapod_msgs/Pose.h>
#include <hexapod_msgs/LegsJoints.h>

class Kinematics {
public:
    /// Creates an instance of a Kinematics object.
    explicit Kinematics();

    /// Calculates the legs joints angles given body & feet position and orientations,
    /// \return The legs joints message.
    hexapod_msgs::LegsJoints body_feet_config_to_legs_joints(hexapod_msgs::Pose body, hexapod_msgs::FeetPositions feet_positions);
};


#endif //HEXAPOD_WS_KINEMATICS_H
