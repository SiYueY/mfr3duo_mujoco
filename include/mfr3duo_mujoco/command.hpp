#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mfr3duo_mujoco {

inline constexpr std::size_t kArmJointCount = 7;

/**
 * @brief Supported control modes for the active spine and FR3 joints.
 */
enum class JointControlMode : std::uint8_t {
    Hybrid = 0,
    Position = 1,
    Velocity = 2,
    Effort = 3,
};

/**
 * @brief Command for one active joint.
 *
 * Position, velocity and effort are interpreted according to mode. Stiffness and
 * damping are used only by Hybrid mode.
 */
struct JointCommand {
    JointControlMode mode{JointControlMode::Position};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double stiffness{0.0};
    double damping{0.0};
};

/**
 * @brief Command for one seven-axis FR3 arm.
 */
struct ArmCommand {
    std::array<JointCommand, kArmJointCount> joints{};
};

/**
 * @brief Command for the vertical spine joint.
 */
using SpineCommand = JointCommand;

/**
 * @brief Command for one Franka Hand.
 */
struct GripperCommand {
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};

/**
 * @brief Planar velocity command for the TMR mobile base.
 */
struct BaseCommand {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

}  // namespace mfr3duo_mujoco
