#pragma once

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
 * @brief State of one active joint.
 */
struct JointState {
    double timestamp{0.0};
    JointControlMode mode{JointControlMode::Position};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};

}  // namespace mfr3duo_mujoco
