#pragma once

#include "mfr3duo_mujoco/data/joint.hpp"

namespace mfr3duo_mujoco {

/** @brief Targets for the four actuated TMR joints. */
struct TmrCommand {
    double front_steering_position{0.0};
    double front_drive_velocity{0.0};
    double rear_steering_position{0.0};
    double rear_drive_velocity{0.0};
};

/** @brief State of the four actuated TMR joints. */
struct TmrState {
    double timestamp{0.0};
    JointState front_steering;
    JointState front_drive;
    JointState rear_steering;
    JointState rear_drive;
};

/** @brief Position and velocity of one unactuated joint. */
struct PassiveJointState {
    double position{0.0};
    double velocity{0.0};
};

/** @brief Simulation-only state of the TMR's unactuated joints. */
struct TmrPassiveState {
    double timestamp{0.0};
    PassiveJointState rocker_arm;
    PassiveJointState front_caster_steering;
    PassiveJointState front_caster_wheel;
    PassiveJointState rear_caster_steering;
    PassiveJointState rear_caster_wheel;
};

}  // namespace mfr3duo_mujoco
