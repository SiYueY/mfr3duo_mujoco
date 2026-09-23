#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "mfr3duo_mujoco/command.hpp"

namespace mfr3duo_mujoco {

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct Pose {
    Vector3 position;
    Quaternion orientation;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
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

/**
 * @brief State of one seven-axis FR3 arm.
 */
struct ArmState {
    std::array<JointState, kArmJointCount> joints{};
};

using SpineState = JointState;

/**
 * @brief State of one Franka Hand.
 */
struct GripperState {
    double timestamp{0.0};
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool stalled{false};
};

/**
 * @brief Ground-truth state of the TMR mobile base.
 */
struct BaseState {
    double timestamp{0.0};
    Pose pose;
    Twist twist;
};

/**
 * @brief State reported by the base IMU.
 */
struct ImuState {
    std::uint64_t sequence{0};
    double timestamp{0.0};
    std::string frame_id;
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
};

/**
 * @brief Coherent low-bandwidth whole-robot state snapshot.
 *
 * Camera and LiDAR samples are intentionally excluded and are read through
 * dedicated APIs so control loops do not copy large sensor payloads.
 */
struct RobotState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    double simulation_time{0.0};
    std::uint64_t step{0};

    SpineState spine;
    ArmState left_arm;
    ArmState right_arm;
    GripperState left_gripper;
    GripperState right_gripper;
    BaseState base;
};

}  // namespace mfr3duo_mujoco
