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
            case 'i':
                twist[3] = options_.cartesian_angular_velocity;
                break;
            case 'k':
                twist[3] = -options_.cartesian_angular_velocity;
                break;
            case 'j':
                twist[4] = options_.cartesian_angular_velocity;
                break;
            case 'l':
                twist[4] = -options_.cartesian_angular_velocity;
                break;
            case 'u':
                twist[5] = options_.cartesian_angular_velocity;
                break;
            case 'o':
                twist[5] = -options_.cartesian_angular_velocity;
                break;
            default:
                break;
        }

        RobotState state;
        if (!simulation_->read_state(state)) return false;

        std::array<double, kArmJointCount> measured{};
        const ArmState& arm_state =
            arm == Arm::Left ? state.left_arm : state.right_arm;
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            measured[index] = arm_state.joints[index].position;
        }

        ArmJacobian source;
        if (!kinematics_->compute_jacobian(
                arm,
                state.spine.position,
                measured,
                cartesian_frame_,
                source)) {
            return false;
        }

        Eigen::Matrix<double, 6, kArmJointCount> jacobian;
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t column = 0; column < kArmJointCount; ++column) {
                jacobian(
                    static_cast<Eigen::Index>(row),
                    static_cast<Eigen::Index>(column)) = source(row, column);
            }
        }

        Eigen::Matrix<double, 6, 6> normal =
            jacobian * jacobian.transpose();
        normal.diagonal().array() +=
            options_.cartesian_damping * options_.cartesian_damping;

        const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(normal);
        if (solver.info() != Eigen::Success) return false;

        const Eigen::Matrix<double, kArmJointCount, 1> solution =
            jacobian.transpose() * solver.solve(twist);
        if (!solution.allFinite()) return false;

        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            const double limit = std::min(
                options_.arm_joint_velocity,
                std::abs(limits.velocity[index]));
            velocity[index] = std::clamp(solution[index], -limit, limit);
        }
    }

    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        target[index] += velocity[index] * dt;
        target[index] = clamp_joint_target(
            target[index],
            limits.lower[index],
            limits.upper[index],
            options_.joint_limit_margin);
    }

    return write_arm(arm, target);
}

bool Teleop::update_spine(char key, double dt) {
    if (key == 'w') spine_target_ += options_.spine_velocity * dt;
    if (key == 's') spine_target_ -= options_.spine_velocity * dt;

    spine_target_ = std::clamp(spine_target_, kSpineMin, kSpineMax);
    return write_spine();
}

bool Teleop::update_gripper(
    Gripper gripper, char key, double dt, double& target) {
    if (key == 'o') target += options_.gripper_velocity * dt;
    if (key == 'c') target -= options_.gripper_velocity * dt;

    target = std::clamp(target, kGripperMin, kGripperMax);
    return write_gripper(gripper, target);
}

bool Teleop::write_arm(
    Arm arm, const std::array<double, kArmJointCount>& target) {
    ArmCommand command;
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        command.joints[index].mode = JointControlMode::Position;
        command.joints[index].position = target[index];
    }
    return simulation_->write_command(arm, command);
}

bool Teleop::write_spine() {
    SpineCommand command;
    command.mode = JointControlMode::Position;
    command.position = spine_target_;
    return simulation_->write_command(command);
}

bool Teleop::write_gripper(Gripper gripper, double target) {
    GripperCommand command;
    command.width = target;
    command.velocity = options_.gripper_velocity;
    command.effort = options_.gripper_effort;
    return simulation_->write_command(gripper, command);
}

bool Teleop::is_motion_key(
    Target target, ArmMode arm_mode, char key) {
    switch (target) {
        case Target::Base:
            return key == 'w' || key == 's' || key == 'a' ||
                   key == 'd' || key == 'q' || key == 'e';
        case Target::LeftArm:
        case Target::RightArm:
            if (arm_mode == ArmMode::Joint) {
                return key == '-' || key == '=';
            }
            return key == 'w' || key == 's' || key == 'a' ||
                   key == 'd' || key == 'r' || key == 'f' ||
                   key == 'i' || key == 'k' || key == 'j' ||
                   key == 'l' || key == 'u' || key == 'o';
        case Target::Spine:
            return key == 'w' || key == 's';
        case Target::LeftGripper:
        case Target::RightGripper:
            return key == 'o' || key == 'c';
    }
    return false;
}

void Teleop::print_help() const {
    std::cout
        << "\nMFR3Duo Teleop\n"
        << "---------------\n"
        << "Targets:\n"
        << "  B  Base             L  Left arm\n"
        << "  R  Right arm        S  Spine\n"
        << "  G  Left gripper     H  Right gripper\n"
        << "\n"
        << "Base:\n"
        << "  w/s  +/-X   a/d  +/-Y   q/e  +/-Yaw\n"
        << "\n"
        << "Arm joint mode:\n"
        << "  1..7  Select joint   -/=  Jog -/+\n"
        << "\n"
        << "Arm Cartesian mode:\n"
        << "  w/s  +/-X   a/d  +/-Y   r/f  +/-Z\n"
        << "  i/k  +/-Roll  j/l  +/-Pitch  u/o  +/-Yaw\n"
        << "  M    Toggle Joint/Cartesian mode\n"
        << "  F    Toggle Base/Tool Cartesian frame\n"
        << "\n"
        << "Spine:    w/s  Up/Down\n"
        << "Gripper:  o/c  Open/Close\n"
        << "\n"
        << "Space   Stop robot motion\n"
        << "Ctrl-R  Reset simulation and resynchronize targets\n"
        << "Esc     Exit\n\n";
}

void Teleop::print_status() const {
    const char* name = "Unknown";
    switch (target_) {
        case Target::Base:
            name = "Base";
            break;
        case Target::LeftArm:
            name = "Left Arm";
            break;
        case Target::RightArm:
            name = "Right Arm";
            break;
        case Target::Spine:
            name = "Spine";
            break;
        case Target::LeftGripper:
            name = "Left Gripper";
            break;
        case Target::RightGripper:
            name = "Right Gripper";
            break;
    }

    std::cout << "Target: " << name;

    if (target_ == Target::LeftArm || target_ == Target::RightArm) {
        std::cout << " | Mode: "
                  << (arm_mode_ == ArmMode::Joint ? "Joint" : "Cartesian");
        if (arm_mode_ == ArmMode::Joint) {
            std::cout << " | Joint: " << (selected_joint_ + 1);
        } else {
            std::cout << " | Frame: "
                      << (cartesian_frame_ == CartesianFrame::Base
                              ? "Base"
                              : "Tool");
        }
    }
    std::cout << '\n';
}

}  // namespace mfr3duo_mujoco::teleop