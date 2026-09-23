#pragma once

#include <array>
#include <chrono>
#include <cstddef>

#include "kinematics.hpp"
#include "mfr3duo_mujoco/simulation.hpp"

namespace mfr3duo_mujoco::teleop {

class Teleop {
public:
    struct Options {
        double arm_joint_velocity{0.30};
        double cartesian_linear_velocity{0.10};
        double cartesian_angular_velocity{0.50};
        double cartesian_damping{0.05};
        double spine_velocity{0.03};
        double gripper_velocity{0.04};
        double gripper_effort{40.0};
        double base_linear_velocity{0.20};
        double base_angular_velocity{0.50};
        std::chrono::milliseconds input_timeout{250};
        double joint_limit_margin{0.01};
    };

    Teleop() = default;

    bool initialize(
        Simulation& simulation,
        Kinematics& kinematics);

    bool initialize(
        Simulation& simulation,
        Kinematics& kinematics,
        const Options& options);

    bool handle_key(
        char key,
        std::chrono::steady_clock::time_point now,
        bool& exit_requested);

    bool update(
        std::chrono::steady_clock::time_point now,
        double dt);

    bool stop_motion();
    bool synchronize();

    void print_help() const;
    void print_status() const;

private:
    enum class Target {
        Base,
        LeftArm,
        RightArm,
        Spine,
        LeftGripper,
        RightGripper,
    };

    enum class ArmMode {
        Joint,
        Cartesian,
    };

    bool update_base(char key);
    bool update_arm(
        Arm arm,
        char key,
        double dt,
        std::array<double, kArmJointCount>& target,
        const ArmJointLimits& limits);
    bool update_spine(char key, double dt);
    bool update_gripper(
        Gripper gripper,
        char key,
        double dt,
        double& target);

    bool write_arm(
        Arm arm,
        const std::array<double, kArmJointCount>& target);
    bool write_spine();
    bool write_gripper(Gripper gripper, double target);

    static bool is_motion_key(
        Target target,
        ArmMode arm_mode,
        char key);

    Simulation* simulation_{nullptr};
    Kinematics* kinematics_{nullptr};
    Options options_{};

    Target target_{Target::Base};
    ArmMode arm_mode_{ArmMode::Joint};
    CartesianFrame cartesian_frame_{CartesianFrame::Base};
    std::size_t selected_joint_{0};

    std::array<double, kArmJointCount> left_arm_target_{};
    std::array<double, kArmJointCount> right_arm_target_{};
    ArmJointLimits left_arm_limits_{};
    ArmJointLimits right_arm_limits_{};

    double spine_target_{0.0};
    double left_gripper_target_{0.0};
    double right_gripper_target_{0.0};

    char active_key_{0};
    std::chrono::steady_clock::time_point input_deadline_{};
};

}  // namespace mfr3duo_mujoco::teleop