#include "teleop.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace mfr3duo_mujoco::teleop {
namespace {

constexpr double kSpineMin = 0.0;
constexpr double kSpineMax = 0.85;
constexpr double kGripperMin = 0.0;
constexpr double kGripperMax = 0.08;
constexpr double kMaxUpdatePeriod = 0.05;

double clamp_joint_target(
    double value, double lower, double upper, double margin) {
    if (!std::isfinite(lower) || !std::isfinite(upper)) return value;

    double min_value = lower + margin;
    double max_value = upper - margin;
    if (min_value > max_value) {
        min_value = lower;
        max_value = upper;
    }
    return std::clamp(value, min_value, max_value);
}

}  // namespace

bool Teleop::initialize(Simulation& simulation, Kinematics& kinematics) {
    return initialize(simulation, kinematics, Options{});
}

bool Teleop::initialize(
    Simulation& simulation, Kinematics& kinematics, const Options& options) {
    if (!std::isfinite(options.arm_joint_velocity) ||
        options.arm_joint_velocity <= 0.0 ||
        !std::isfinite(options.cartesian_linear_velocity) ||
        options.cartesian_linear_velocity <= 0.0 ||
        !std::isfinite(options.cartesian_angular_velocity) ||
        options.cartesian_angular_velocity <= 0.0 ||
        !std::isfinite(options.cartesian_damping) ||
        options.cartesian_damping <= 0.0 ||
        !std::isfinite(options.spine_velocity) ||
        options.spine_velocity <= 0.0 ||
        !std::isfinite(options.gripper_velocity) ||
        options.gripper_velocity <= 0.0 ||
        !std::isfinite(options.gripper_effort) ||
        options.gripper_effort < 0.0 ||
        !std::isfinite(options.base_linear_velocity) ||
        options.base_linear_velocity <= 0.0 ||
        !std::isfinite(options.base_angular_velocity) ||
        options.base_angular_velocity <= 0.0 ||
        options.input_timeout <= std::chrono::milliseconds::zero() ||
        !std::isfinite(options.joint_limit_margin) ||
        options.joint_limit_margin < 0.0) {
        return false;
    }

    simulation_ = &simulation;
    kinematics_ = &kinematics;
    options_ = options;

    if (!kinematics_->read_limits(Arm::Left, left_arm_limits_) ||
        !kinematics_->read_limits(Arm::Right, right_arm_limits_)) {
        return false;
    }

    return synchronize();
}

bool Teleop::handle_key(
    char key,
    std::chrono::steady_clock::time_point now,
    bool& exit_requested) {
    exit_requested = false;

    if (key == 27) {
        exit_requested = true;
        return true;
    }
    if (key == ' ') {
        return stop_motion();
    }
    if (key == 18) {
        if (simulation_ == nullptr || !simulation_->reset()) return false;
        return stop_motion();
    }

    Target requested_target = target_;
    bool target_change = false;
    switch (key) {
        case 'B':
            requested_target = Target::Base;
            target_change = requested_target != target_;
            break;
        case 'L':
            requested_target = Target::LeftArm;
            target_change = requested_target != target_;
            break;
        case 'R':
            requested_target = Target::RightArm;
            target_change = requested_target != target_;
            break;
        case 'S':
            requested_target = Target::Spine;
            target_change = requested_target != target_;
            break;
        case 'G':
            requested_target = Target::LeftGripper;
            target_change = requested_target != target_;
            break;
        case 'H':
            requested_target = Target::RightGripper;
            target_change = requested_target != target_;
            break;
        default:
            break;
    }

    if (target_change) {
        if (!stop_motion()) return false;
        target_ = requested_target;
        print_status();
        return true;
    }

    const bool arm_selected =
        target_ == Target::LeftArm || target_ == Target::RightArm;

    if (arm_selected && key == 'M') {
        if (!stop_motion()) return false;
        arm_mode_ =
            arm_mode_ == ArmMode::Joint ? ArmMode::Cartesian : ArmMode::Joint;
        print_status();
        return true;
    }

    if (arm_selected && arm_mode_ == ArmMode::Cartesian && key == 'F') {
        active_key_ = 0;
        cartesian_frame_ =
            cartesian_frame_ == CartesianFrame::Base
                ? CartesianFrame::Tool
                : CartesianFrame::Base;
        print_status();
        return true;
    }

    if (arm_selected && arm_mode_ == ArmMode::Joint &&
        key >= '1' && key <= '7') {
        active_key_ = 0;
        selected_joint_ = static_cast<std::size_t>(key - '1');
        print_status();
        return true;
    }

    if (is_motion_key(target_, arm_mode_, key)) {
        active_key_ = key;
        input_deadline_ = now + options_.input_timeout;
    }

    return true;
}

bool Teleop::update(
    std::chrono::steady_clock::time_point now,
    double dt) {
    if (simulation_ == nullptr || kinematics_ == nullptr ||
        !std::isfinite(dt) || dt < 0.0) {
        return false;
    }

    dt = std::min(dt, kMaxUpdatePeriod);
    if (now >= input_deadline_) active_key_ = 0;

    switch (target_) {
        case Target::Base:
            return update_base(active_key_);
        case Target::LeftArm:
            return update_arm(
                Arm::Left,
                active_key_,
                dt,
                left_arm_target_,
                left_arm_limits_);
        case Target::RightArm:
            return update_arm(
                Arm::Right,
                active_key_,
                dt,
                right_arm_target_,
                right_arm_limits_);
        case Target::Spine:
            return update_spine(active_key_, dt);
        case Target::LeftGripper:
            return update_gripper(
                Gripper::Left, active_key_, dt, left_gripper_target_);
        case Target::RightGripper:
            return update_gripper(
                Gripper::Right, active_key_, dt, right_gripper_target_);
    }
    return false;
}

bool Teleop::stop_motion() {
    if (simulation_ == nullptr || !synchronize()) return false;

    active_key_ = 0;

    BaseCommand base;
    return simulation_->write_command(base) &&
           write_arm(Arm::Left, left_arm_target_) &&
           write_arm(Arm::Right, right_arm_target_) &&
           write_spine() &&
           write_gripper(Gripper::Left, left_gripper_target_) &&
           write_gripper(Gripper::Right, right_gripper_target_);
}

bool Teleop::synchronize() {
    if (simulation_ == nullptr) return false;

    RobotState state;
    if (!simulation_->read_state(state)) return false;

    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        left_arm_target_[index] = state.left_arm.joints[index].position;
        right_arm_target_[index] = state.right_arm.joints[index].position;
    }
    spine_target_ = state.spine.position;
    left_gripper_target_ = state.left_gripper.width;
    right_gripper_target_ = state.right_gripper.width;
    active_key_ = 0;
    input_deadline_ = {};
    return true;
}

bool Teleop::update_base(char key) {
    BaseCommand command;
    switch (key) {
        case 'w':
            command.linear_x = options_.base_linear_velocity;
            break;
        case 's':
            command.linear_x = -options_.base_linear_velocity;
            break;
        case 'a':
            command.linear_y = options_.base_linear_velocity;
            break;
        case 'd':
            command.linear_y = -options_.base_linear_velocity;
            break;
        case 'q':
            command.angular_z = options_.base_angular_velocity;
            break;
        case 'e':
            command.angular_z = -options_.base_angular_velocity;
            break;
        default:
            break;
    }
    return simulation_->write_command(command);
}

bool Teleop::update_arm(
    Arm arm,
    char key,
    double dt,
    std::array<double, kArmJointCount>& target,
    const ArmJointLimits& limits) {
    std::array<double, kArmJointCount> velocity{};

    if (arm_mode_ == ArmMode::Joint) {
        const double limit = std::min(
            options_.arm_joint_velocity,
            std::abs(limits.velocity[selected_joint_]));
        if (key == '-') velocity[selected_joint_] = -limit;
        if (key == '=') velocity[selected_joint_] = limit;
    } else if (key != 0) {
        Eigen::Matrix<double, 6, 1> twist =
            Eigen::Matrix<double, 6, 1>::Zero();

        switch (key) {
            case 'w':
                twist[0] = options_.cartesian_linear_velocity;
                break;
            case 's':
                twist[0] = -options_.cartesian_linear_velocity;
                break;
            case 'a':
                twist[1] = options_.cartesian_linear_velocity;
                break;
            case 'd':
                twist[1] = -options_.cartesian_linear_velocity;
                break;
            case 'r':
                twist[2] = options_.cartesian_linear_velocity;
                break;
            case 'f':
                twist[2] = -options_.cartesian_linear_velocity;
                break;