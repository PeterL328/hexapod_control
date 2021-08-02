//
// Created by peter on 2021-07-31.
//

#ifndef HEXAPOD_WS_GAIT_H
#define HEXAPOD_WS_GAIT_H

#include <geometry_msgs/Twist.h>

#include "hexapod_model.h"

class Gait {
public:
    enum class Mode {
        Tripod
    };

    /// Creates an instance of a Gait object.
    /// \model body A shared pointer to the hexapod model.
    /// \param publish_rate The publish rate of the controller.
    explicit Gait(std::shared_ptr<HexapodModel> model, float publish_rate);

    /// Updates the current gait mode.
    /// \param new_mode The new gait mode.
    void UpdateGaitMode(Mode new_mode);

    /// Updates the body position and orientation and feet positions based on the twist command.
    /// \param twist The twist message.
    void UpdateModel(geometry_msgs::Twist& twist);

private:
    Mode gait_mode_;
    std::shared_ptr<HexapodModel> hexapod_model_;
    int period_cycle_length_{0};
    int previous_period_cycle_length_{0};
    int period_cycle_{0};
    float publish_rate_;

    // Loads from parameter server
    float leg_walk_distance_;
};


#endif //HEXAPOD_WS_GAIT_H
