//
// Created by peter on 2021-06-07.
//

#include <cmath>
#include <Eigen/Geometry>

#include "kinematics.h"

#define _USE_MATH_DEFINES

using namespace Eigen;

Kinematics::Kinematics(std::shared_ptr<HexapodModel> model) : hexapod_model_(model){
}

hexapod_msgs::LegsJoints Kinematics::body_feet_config_to_legs_joints(hexapod_msgs::Pose body, hexapod_msgs::FeetPositions feet_positions) {
    // Message for return
    hexapod_msgs::LegsJoints msg;

    // General definitions
    float coxa_length = hexapod_model_->get_coxa_length();
    float femur_length = hexapod_model_->get_femur_length();
    float tibia_length = hexapod_model_->get_tibia_length();
    Vector3f global_origin_to_body_origin(body.position.x, body.position.y, body.position.z);
    Matrix3f body_rot_mat = euler_angles_to_rotation_matrix(body.orientation.roll, body.orientation.yaw, body.orientation.pitch);

    // Index in the following order: ['LF', 'LM', 'LB', 'RF', 'RM', 'RB']
    for (int i = 0; i < feet_positions.foot.size(); i++) {
        Vector3f body_origin_to_coxa_joint_in_local = hexapod_model_->get_center_to_coxa(i);
        Vector3f global_origin_to_feet_ground_contact_point(feet_positions.foot[i].x, feet_positions.foot[i].y, feet_positions.foot[i].z);

        Vector3f feet_ground_contact_point_to_coxa = global_origin_to_body_origin + (body_rot_mat * body_origin_to_coxa_joint_in_local) - global_origin_to_feet_ground_contact_point;
        float coxa_angle_rad = atan(feet_ground_contact_point_to_coxa[1] / feet_ground_contact_point_to_coxa[0]);

        int leg_dir = i <= 2 ? -1 : 1;
        Vector3f body_origin_to_femur_joint_in_local(
            body_origin_to_coxa_joint_in_local[0] + (leg_dir * coxa_length * cos(coxa_angle_rad)),
            body_origin_to_coxa_joint_in_local[1] + (leg_dir * coxa_length * sin(coxa_angle_rad)),
            body_origin_to_coxa_joint_in_local[2]);

        Vector3f feet_ground_contact_point_to_femur = global_origin_to_body_origin + (body_rot_mat * body_origin_to_femur_joint_in_local) - global_origin_to_feet_ground_contact_point;

        float rho_angle = atan(feet_ground_contact_point_to_femur[2] / sqrt(pow(feet_ground_contact_point_to_femur[0], 2) + pow(feet_ground_contact_point_to_femur[1], 2)));
        float phi_angle = asin((feet_ground_contact_point_to_femur[2] - feet_ground_contact_point_to_coxa[2])/ coxa_length);

        float femur_angle_rad = acos((pow(femur_length, 2) + feet_ground_contact_point_to_femur.squaredNorm() - pow(tibia_length, 2)) / (2.f * femur_length * feet_ground_contact_point_to_femur.norm())) - (rho_angle + phi_angle);
        float tibia_angle_rad = M_PI_2 - acos((pow(femur_length, 2) - feet_ground_contact_point_to_femur.squaredNorm() + pow(tibia_length, 2)) / (2.f * femur_length * tibia_length));

        switch (i) {
            case 0:
                msg.left_front_leg.coxa = coxa_angle_rad;
                msg.left_front_leg.femur = femur_angle_rad;
                msg.left_front_leg.tibia = tibia_angle_rad;
                break;
            case 1:
                msg.left_mid_leg.coxa = coxa_angle_rad;
                msg.left_mid_leg.femur = femur_angle_rad;
                msg.left_mid_leg.tibia = tibia_angle_rad;
                break;
            case 2:
                msg.left_back_leg.coxa = coxa_angle_rad;
                msg.left_back_leg.femur = femur_angle_rad;
                msg.left_back_leg.tibia = tibia_angle_rad;
                break;
            case 3:
                msg.right_front_leg.coxa = coxa_angle_rad;
                msg.right_front_leg.femur = femur_angle_rad;
                msg.right_front_leg.tibia = tibia_angle_rad;
                break;
            case 4:
                msg.right_mid_leg.coxa = coxa_angle_rad;
                msg.right_mid_leg.femur = femur_angle_rad;
                msg.right_mid_leg.tibia = tibia_angle_rad;
                break;
            case 5:
                msg.right_back_leg.coxa = coxa_angle_rad;
                msg.right_back_leg.femur = femur_angle_rad;
                msg.right_back_leg.tibia = tibia_angle_rad;
                break;
        }
    }
    return msg;
}

Matrix3f Kinematics::euler_angles_to_rotation_matrix(float roll, float yaw, float pitch) {
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());

    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;

    return q.matrix();
}