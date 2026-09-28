#pragma once

#include <cstdint>

#include "mfr3duo_mujoco/data/arm.hpp"
#include "mfr3duo_mujoco/data/base.hpp"
#include "mfr3duo_mujoco/data/gripper.hpp"
#include "mfr3duo_mujoco/data/spine.hpp"

namespace mfr3duo_mujoco {

/**
 * @brief Coherent whole-robot motion state from one simulation snapshot.
 *
 * Sensor samples are read through dedicated APIs.
 */
struct RobotState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    double simulation_time{0.0};
    std::uint64_t step{0};

    BaseState base;
    SpineState spine;
    ArmState left_arm;
    ArmState right_arm;
    GripperState left_gripper;
    GripperState right_gripper;
};

/**
 * @brief Complete motion command, published as one command-buffer update.
 *
 * Every member is submitted, including members left at their default values.
 * Use device-level write_command overloads for partial updates.
 */
struct RobotCommand {
    BaseCommand base;
    SpineCommand spine;
    ArmCommand left_arm;
    ArmCommand right_arm;
    GripperCommand left_gripper;
    GripperCommand right_gripper;
};

}  // namespace mfr3duo_mujoco
