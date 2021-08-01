//
// Created by peter on 2021-07-31.
//

#include "gait.h"

Gait::Gait(std::shared_ptr<HexapodModel> model) : hexapod_model_(model), gait_mode_(Mode::Tripod){
}

void Gait::UpdateGaitMode(Mode new_mode) {
    gait_mode_ = new_mode;
}

void Gait::UpdateModel(geometry_msgs::Twist& twist) {
    // TODO: fill in this function
}