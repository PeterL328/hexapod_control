//
// Created by peter on 2021-10-17.
//

#ifndef HEXAPOD_WS_GAIT_H
#define HEXAPOD_WS_GAIT_H

#include <unordered_map>
#include <vector>

class Gait {
public:
    enum class Mode {
        Tripod,
        Ripple,
        Wave
    };

    /// Creates an instance of a Gait object.
    /// \param initial_gait The initial gait mode.
    explicit Gait(Mode initial_gait);

    /// Updates the current gait mode and resets the sequence index.
    /// \param new_mode The new gait mode.
    void update_gait_mode(Mode new_mode);

    /// Gets the current gait sequence and next.
    /// \return The current sequence.
    std::vector<int> get_current_seq_and_next();

    /// Gets the duty factor of the gait.
    /// \return The duty factor.
    float get_duty_factor();

    /// Gets the sequence size.
    /// \return The sequence size.
    int get_sequence_size();

    /// Gets the sequence index.
    /// \return The sequence index.
    int get_sequence_index();
private:
    Mode current_mode_;
    int current_seq_idx_{0};
    std::unordered_map<Mode, std::vector<std::vector<int>>> gait_sequence_mapping_;
    std::unordered_map<Mode, float> duty_factor_mapping_;

    /// Configures the gait sequencing and duty factor mapping.
    void configure_mapping();
};


#endif //HEXAPOD_WS_GAIT_H
