#include <cmath>
#include <cstdlib>
#include <iostream>
#include <type_traits>
#include <utility>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

using namespace mfr3duo_mujoco;

template <typename T, typename = void>
struct HasPassive : std::false_type {};
template <typename T>
struct HasPassive<T, std::void_t<decltype(std::declval<T>().passive)>>
    : std::true_type {};
static_assert(!HasPassive<TmrState>::value);
static_assert(!HasPassive<RobotState>::value);

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

bool same_joint(const JointState& lhs, const JointState& rhs) {
    return lhs.timestamp == rhs.timestamp && lhs.mode == rhs.mode &&
           lhs.position == rhs.position && lhs.velocity == rhs.velocity &&
           lhs.effort == rhs.effort;
}

bool same_arm(const ArmState& lhs, const ArmState& rhs) {
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        if (!same_joint(lhs.joints[index], rhs.joints[index])) return false;
    }
    return true;
}

bool same_gripper(const GripperState& lhs, const GripperState& rhs) {
    return lhs.timestamp == rhs.timestamp && lhs.width == rhs.width &&
           lhs.velocity == rhs.velocity && lhs.effort == rhs.effort &&
           lhs.stalled == rhs.stalled;
}

bool same_tmr(const TmrState& lhs, const TmrState& rhs) {
    return lhs.timestamp == rhs.timestamp &&
           same_joint(lhs.front_steering, rhs.front_steering) &&
           same_joint(lhs.front_drive, rhs.front_drive) &&
           same_joint(lhs.rear_steering, rhs.rear_steering) &&
           same_joint(lhs.rear_drive, rhs.rear_drive);
}

bool check_snapshot(Simulation& simulation, RobotState& robot) {
    ArmState left_arm;
    ArmState right_arm;
    SpineState spine;
    GripperState left_gripper;
    GripperState right_gripper;
    TmrState tmr;
    if (!check(simulation.read_state(robot), "read robot") ||
        !check(simulation.read_state(Arm::Left, left_arm), "read left arm") ||
        !check(simulation.read_state(Arm::Right, right_arm), "read right arm") ||
        !check(simulation.read_state(spine), "read spine") ||
        !check(simulation.read_state(Gripper::Left, left_gripper), "read left gripper") ||
        !check(simulation.read_state(Gripper::Right, right_gripper), "read right gripper") ||
        !check(simulation.read_state(tmr), "read tmr")) return false;
    return check(same_arm(robot.left_arm, left_arm), "left arm snapshot mismatch") &&
           check(same_arm(robot.right_arm, right_arm), "right arm snapshot mismatch") &&
           check(same_joint(robot.spine, spine), "spine snapshot mismatch") &&
           check(same_gripper(robot.left_gripper, left_gripper),
                 "left gripper snapshot mismatch") &&
           check(same_gripper(robot.right_gripper, right_gripper),
                 "right gripper snapshot mismatch") &&
           check(same_tmr(robot.tmr, tmr), "tmr snapshot mismatch");
}

JointCommand hold(const JointState& state) {
    JointCommand command;
    command.position = state.position;
    return command;
}

RobotCommand make_motion_command(const RobotState& state) {
    RobotCommand command;
    command.spine = hold(state.spine);
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        command.left_arm.joints[index] = hold(state.left_arm.joints[index]);
        command.right_arm.joints[index] = hold(state.right_arm.joints[index]);
    }
    command.left_gripper = {
        state.left_gripper.width > 0.04 ? 0.02 : 0.06, 0.05, 5.0};
    command.right_gripper = {
        state.right_gripper.width > 0.04 ? 0.02 : 0.06, 0.05, 5.0};
    command.tmr.front_steering_position = state.tmr.front_steering.position + 0.3;
    command.tmr.rear_steering_position = state.tmr.rear_steering.position - 0.3;
    command.tmr.front_drive_velocity = 4.0;
    command.tmr.rear_drive_velocity = 4.0;
    return command;
}

bool check_passive(Simulation& simulation) {
    TmrPassiveState state;
    if (!check(simulation.read_state(state), "read passive TMR")) return false;
    return check(std::isfinite(state.rocker_arm.position) &&
                     std::isfinite(state.front_caster_steering.velocity) &&
                     std::isfinite(state.front_caster_wheel.position) &&
                     std::isfinite(state.rear_caster_steering.position) &&
                     std::isfinite(state.rear_caster_wheel.velocity),
                 "non-finite passive TMR state");
}

bool run_case(const SimulationOptions& options) {
    Simulation simulation;
    if (!check(simulation.initialize(MFR3DUO_TEST_SCENE_PATH, options), "initialize"))
        return false;
    bool passed = check(simulation.step(), "initial step");
    RobotState initial;
    if (passed) passed = check_snapshot(simulation, initial) && check_passive(simulation);

    if (passed && !options.imu_enabled) {
        ImuState imu;
        passed = check(!simulation.read_state(imu), "disabled IMU read succeeded");
    }

    if (passed) {
        RobotCommand command = make_motion_command(initial);
        for (std::size_t index = 0; index < kArmJointCount; ++index) {
            command.left_arm.joints[index].mode = JointControlMode::Velocity;
            command.right_arm.joints[index].mode = JointControlMode::Effort;
        }
        command.left_arm.joints[0].velocity = 0.05;
        for (int iteration = 0; iteration < 500 && passed; ++iteration) {
            passed = check(simulation.write_command(command), "whole robot write") &&
                     check(simulation.step(), "whole robot step");
        }

        RobotState after_batch;
        if (passed) passed = check_snapshot(simulation, after_batch);
        if (passed) {
            passed = check(after_batch.spine.mode == JointControlMode::Position,
                           "spine mode after batch") &&
                     check(after_batch.tmr.front_steering.mode == JointControlMode::Position,
                           "front steering mode") &&
                     check(after_batch.tmr.front_drive.mode == JointControlMode::Velocity,
                           "front drive mode") &&
                     check(after_batch.tmr.rear_steering.mode == JointControlMode::Position,
                           "rear steering mode") &&
                     check(after_batch.tmr.rear_drive.mode == JointControlMode::Velocity,
                           "rear drive mode");
            for (std::size_t index = 0; index < kArmJointCount && passed; ++index) {
                passed =
                    check(after_batch.left_arm.joints[index].mode ==
                              JointControlMode::Velocity,
                          "left arm mode after batch") &&
                    check(after_batch.right_arm.joints[index].mode ==
                              JointControlMode::Effort,
                          "right arm mode after batch");
            }
            passed = passed &&
                     check(after_batch.tmr.front_steering.position >
                               initial.tmr.front_steering.position + 0.05,
                           "front steering did not move positive") &&
                     check(after_batch.tmr.rear_steering.position <
                               initial.tmr.rear_steering.position - 0.05,
                           "rear steering did not move negative") &&
                     check(after_batch.tmr.front_drive.position >
                               initial.tmr.front_drive.position + 0.05,
                           "front drive did not rotate positive") &&
                     check(after_batch.tmr.rear_drive.position >
                               initial.tmr.rear_drive.position + 0.05,
                           "rear drive did not rotate positive") &&
                     check(std::abs(after_batch.left_gripper.width -
                                    command.left_gripper.width) <
                               std::abs(initial.left_gripper.width -
                                        command.left_gripper.width),
                           "left gripper did not approach width target") &&
                     check(std::abs(after_batch.right_gripper.width -
                                    command.right_gripper.width) <
                               std::abs(initial.right_gripper.width -
                                        command.right_gripper.width),
                           "right gripper did not approach width target");
        }

        if (passed) {
            RobotCommand invalid = command;
            invalid.left_arm.joints[0].mode = JointControlMode::Position;
            invalid.left_gripper.velocity = -1.0;
            passed = check(!simulation.write_command(invalid),
                           "invalid whole robot command accepted") &&
                     check(simulation.step(), "step after rejected command");
            RobotState after_rejection;
            if (passed) passed = check_snapshot(simulation, after_rejection);
            if (passed) {
                passed = check(after_rejection.left_arm.joints[0].mode ==
                                   JointControlMode::Velocity,
                               "rejected command partially changed joint mode");
            }
        }

        if (passed) {
            TmrCommand stop = command.tmr;
            stop.front_drive_velocity = 0.0;
            stop.rear_drive_velocity = 0.0;
            stop.front_steering_position = after_batch.tmr.front_steering.position;
            stop.rear_steering_position = after_batch.tmr.rear_steering.position;
            for (int iteration = 0; iteration < 300 && passed; ++iteration) {
                passed = check(simulation.write_command(stop), "device-level TMR write") &&
                         check(simulation.step(), "device-level TMR step");
            }
            RobotState after_stop;
            if (passed) passed = check_snapshot(simulation, after_stop);
            if (passed) {
                passed = check(std::abs(after_stop.tmr.front_drive.velocity) < 0.5 &&
                                   std::abs(after_stop.tmr.rear_drive.velocity) < 0.5,
                               "TMR drives did not stop") &&
                         check(std::abs(after_stop.tmr.front_steering.position -
                                        stop.front_steering_position) < 0.1,
                               "TMR steering did not hold");
            }
            if (passed) {
                stop.front_drive_velocity = -4.0;
                stop.rear_drive_velocity = -4.0;
                for (int iteration = 0; iteration < 300 && passed; ++iteration) {
                    passed = check(simulation.write_command(stop), "reverse TMR write") &&
                             check(simulation.step(), "reverse TMR step");
                }
                TmrState reverse;
                if (passed) passed = check(simulation.read_state(reverse), "reverse TMR read");
                if (passed) {
                    passed = check(reverse.front_drive.position <
                                       after_stop.tmr.front_drive.position - 0.05 &&
                                       reverse.rear_drive.position <
                                       after_stop.tmr.rear_drive.position - 0.05,
                                   "TMR drives did not reverse");
                }
            }
        }
    }
    return check(simulation.shutdown(), "shutdown") && passed;
}

}  // namespace

int main() {
    SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;
    options.lidars_enabled = false;
    options.imu_enabled = false;
    if (!run_case(options)) return EXIT_FAILURE;

    options.imu_enabled = true;
    options.lidars_enabled = true;
    if (!run_case(options)) return EXIT_FAILURE;

    options.cameras_enabled = true;
    options.lidars_enabled = false;
    options.camera_width = 32;
    options.camera_height = 24;
    options.camera_period = 0.04;
    return run_case(options) ? EXIT_SUCCESS : EXIT_FAILURE;
}
