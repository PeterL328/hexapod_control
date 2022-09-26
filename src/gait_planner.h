//
// Created by peter on 2021-07-31.
//

#ifndef HEXAPOD_WS_GAIT_PLANNER_H
#define HEXAPOD_WS_GAIT_PLANNER_H

#include <geometry_msgs/Twist.h>

#include "hexapod_model.h"
#include "gait.h"

class GaitPlanner {
public:
    /// Creates an instance of a GaitPlanner object.
    /// \param model A shared pointer to the hexapod model.
    /// \param publish_rate The publish rate of the controller.
    explicit GaitPlanner(std::shared_ptr<HexapodModel> model, float publish_rate);

    /// Updates the current gait mode.
    /// \param new_mode The new gait mode.
    void update_gait_mode(Gait::Mode new_mode);

    /// Updates the body position and orientation and feet positions based on the twist command.
    /// \param twist The twist message.
    void update_model(geometry_msgs::Twist& twist);

    /// Resets walk.
    void reset_state();

private:
    std::unique_ptr<Gait> gait_;
    std::shared_ptr<HexapodModel> hexapod_model_;

    std::vector<int> gait_seq_;
    // Used for marking which leg has moved in a gait period.
    std::vector<int> legs_moved_;
    int period_cycle_length_{0};
    int previous_period_cycle_length_{0};
    int period_cycle_{0};
    int reset_leg_period_cycle_length_{30};
    float publish_rate_;
    float linear_deadzone_{0.02};
    float angular_deadzone_{0.02};
    bool was_travelling_{false};
    bool is_travelling_{false};
    bool force_extra_period_{false};
    bool is_translating_{true};

    // Loads from parameter server
    float gait_period_distance_;
    float gait_period_rotation_;
    float leg_lift_height_;

    /// Get the vector that is perpendicular to the clockwise.
    Eigen::Vector2f get_perpendicular_clockwise(const Eigen::Vector2f &vec) const;
};


#endif //HEXAPOD_WS_GAIT_PLANNER_H
