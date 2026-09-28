#include <cmath>
#include <cstdlib>
#include <iostream>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

using namespace mfr3duo_mujoco;

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

bool same_base(const BaseState& lhs, const BaseState& rhs) {
    return lhs.timestamp == rhs.timestamp &&
           lhs.pose.position.x == rhs.pose.position.x &&
           lhs.pose.position.y == rhs.pose.position.y &&
           lhs.pose.position.z == rhs.pose.position.z &&
           lhs.pose.orientation.x == rhs.pose.orientation.x &&
           lhs.pose.orientation.y == rhs.pose.orientation.y &&
           lhs.pose.orientation.z == rhs.pose.orientation.z &&
           lhs.pose.orientation.w == rhs.pose.orientation.w &&
           lhs.twist.linear.x == rhs.twist.linear.x &&
           lhs.twist.linear.y == rhs.twist.linear.y &&
           lhs.twist.linear.z == rhs.twist.linear.z &&
           lhs.twist.angular.x == rhs.twist.angular.x &&
           lhs.twist.angular.y == rhs.twist.angular.y &&
           lhs.twist.angular.z == rhs.twist.angular.z;
}

bool check_snapshot(Simulation& simulation, RobotState& robot) {
    ArmState left_arm;
    ArmState right_arm;
    SpineState spine;
    GripperState left_gripper;
    GripperState right_gripper;
    BaseState base;
    if (!check(simulation.read_state(robot), "read robot") ||
        !check(simulation.read_state(Arm::Left, left_arm), "read left arm") ||
        !check(simulation.read_state(Arm::Right, right_arm), "read right arm") ||
        !check(simulation.read_state(spine), "read spine") ||
        !check(simulation.read_state(Gripper::Left, left_gripper), "read left gripper") ||
        !check(simulation.read_state(Gripper::Right, right_gripper), "read right gripper") ||
        !check(simulation.read_state(base), "read base")) {
        return false;
    }
    return check(same_arm(robot.left_arm, left_arm), "left arm snapshot mismatch") &&
           check(same_arm(robot.right_arm, right_arm), "right arm snapshot mismatch") &&
           check(same_joint(robot.spine, spine), "spine snapshot mismatch") &&
           check(same_gripper(robot.left_gripper, left_gripper),
                 "left gripper snapshot mismatch") &&
           check(same_gripper(robot.right_gripper, right_gripper),
                 "right gripper snapshot mismatch") &&
           check(same_base(robot.base, base), "base snapshot mismatch");
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
    command.base.linear_x = 0.2;
    return command;
}

bool run_case(const SimulationOptions& options) {
    Simulation simulation;
    if (!check(simulation.initialize(MFR3DUO_TEST_SCENE_PATH, options), "initialize")) {
        return false;
    }
    bool passed = check(simulation.step(), "initial step");
    RobotState initial;
    if (passed) passed = check_snapshot(simulation, initial);

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
        command.spine.mode = JointControlMode::Hybrid;
        command.spine.stiffness = 100.0;
        command.spine.damping = 5.0;
        passed = check(simulation.write_command(command), "whole robot write") &&
                 check(simulation.step(250), "whole robot step");

        RobotState after_batch;
        if (passed) passed = check_snapshot(simulation, after_batch);
        if (passed) {
            passed = check(after_batch.spine.mode == JointControlMode::Hybrid,
                           "spine mode after batch");
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
                     check(after_batch.base.pose.position.x >
                               initial.base.pose.position.x + 0.005,
                           "base did not follow forward command") &&
                     check(std::abs(after_batch.left_gripper.width -
                                    command.left_gripper.width) <
                               std::abs(initial.left_gripper.width -
                                        command.left_gripper.width),
                           "left gripper did not approach width target") &&
                     check(std::abs(after_batch.right_gripper.width -
                                    command.right_gripper.width) <
                               std::abs(initial.right_gripper.width -
                                        command.right_gripper.width),
                           "right gripper did not approach width target") &&
                     check(std::isfinite(after_batch.left_gripper.velocity) &&
                               std::isfinite(after_batch.right_gripper.effort) &&
                               std::isfinite(after_batch.base.twist.linear.x),
                           "device feedback after batch");
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
            ArmCommand partial = command.left_arm;
            for (auto& joint : partial.joints) {
                joint.mode = JointControlMode::Position;
            }
            passed = check(simulation.write_command(Arm::Left, partial),
                           "device-level arm write") &&
                     check(simulation.step(), "device-level step");
            RobotState after_partial;
            if (passed) passed = check_snapshot(simulation, after_partial);
            if (passed) {
                passed = check(after_partial.left_arm.joints[0].mode ==
                                   JointControlMode::Position,
                               "left arm partial update missing") &&
                         check(after_partial.right_arm.joints[0].mode ==
                                   JointControlMode::Effort,
                               "left arm partial update changed right arm");
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
