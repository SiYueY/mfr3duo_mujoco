#pragma once

#include "mfr3duo_mujoco/data/math.hpp"

namespace mfr3duo_mujoco {

/**
 * @brief Planar velocity command for the TMR mobile base.
 */
struct BaseCommand {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

/**
 * @brief Ground-truth state of the TMR mobile base.
 */
struct BaseState {
    double timestamp{0.0};
    Pose pose;
    Twist twist;
};

}  // namespace mfr3duo_mujoco
