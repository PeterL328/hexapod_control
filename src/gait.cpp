//
// Created by peter on 2021-10-17.
//

#include "gait.h"

Gait::Gait(Mode initial_gait) {
    current_mode_ = initial_gait;
    configure_mapping();
}

void Gait::update_gait_mode(Mode new_mode) {
    current_mode_ = new_mode;
    current_seq_idx_ = 0;
}

std::vector<int> Gait::get_current_seq_and_next() {
    std::vector<int> current_seq = gait_sequence_mapping_[current_mode_][current_seq_idx_];
    current_seq_idx_++;
    current_seq_idx_ %= gait_sequence_mapping_[current_mode_].size();
    return current_seq;
}

float Gait::get_duty_factor() {
    return duty_factor_mapping_[current_mode_];
}

void Gait::configure_mapping() {
    // 1 means the ith leg is in transfer stage.
    // 0 means the ith leg is in support stage.
    gait_sequence_mapping_[Mode::Tripod] = {
        {1, 0, 1, 0, 1, 0},
        {0, 1, 0, 1, 0, 1}
    };
    duty_factor_mapping_[Mode::Tripod] = 0.5f;

    gait_sequence_mapping_[Mode::Ripple] = {
        {0, 0, 1, 0, 1, 0},
        {0, 1, 0, 1, 0, 0},
        {1, 0, 0, 0, 0, 1}
    };
    duty_factor_mapping_[Mode::Ripple] = 0.6667f;

    gait_sequence_mapping_[Mode::Wave] = {
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0},
    };
    duty_factor_mapping_[Mode::Wave] = 0.8333f;
}
