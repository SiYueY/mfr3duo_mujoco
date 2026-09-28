#pragma once

#include <array>

#include "mfr3duo_mujoco/data/joint.hpp"

namespace mfr3duo_mujoco {

/**
 * @brief Command for one seven-axis FR3 arm.
 */
struct ArmCommand {
    std::array<JointCommand, kArmJointCount> joints{};
};

/**
 * @brief State of one seven-axis FR3 arm.
 */
struct ArmState {
    std::array<JointState, kArmJointCount> joints{};
};

}  // namespace mfr3duo_mujoco
