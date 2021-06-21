//
// Created by peter on 2021-06-07.
//

#include "kinematics.h"

Kinematics::Kinematics(std::shared_ptr<HexapodModel> model) {
    hexapod_model_ = model;
}

hexapod_msgs::LegsJoints Kinematics::body_feet_config_to_legs_joints(hexapod_msgs::Pose body, hexapod_msgs::FeetPositions feet_positions) {
    hexapod_msgs::LegsJoints msg;
    return msg;
}
