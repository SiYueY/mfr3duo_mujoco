#include <chrono>
#include <cstdlib>
#include <iostream>

#include "kinematics.hpp"
#include "mfr3duo_mujoco/simulation.hpp"
#include "teleop.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main() {
    using namespace mfr3duo_mujoco;
    using namespace mfr3duo_mujoco::teleop;

    SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;
    options.lidars_enabled = false;
    options.imu_enabled = false;

    Simulation simulation;
    if (!check(simulation.initialize(MFR3DUO_TEST_SCENE_PATH, options), "simulation init")) {
        return EXIT_FAILURE;
    }

    Kinematics kinematics;
    if (!check(kinematics.initialize(MFR3DUO_TEST_URDF_PATH), "kinematics init")) {
        simulation.shutdown();
        return EXIT_FAILURE;
    }

    Teleop teleop;
    if (!check(teleop.initialize(simulation, kinematics), "teleop init")) {
        simulation.shutdown();
        return EXIT_FAILURE;
    }

    const auto now = std::chrono::steady_clock::now();
    bool exit_requested = false;
    bool passed =
        check(teleop.handle_key('L', now, exit_requested), "select left arm") &&
        check(!exit_requested, "unexpected exit request") &&
        check(teleop.handle_key('M', now, exit_requested), "select Cartesian mode") &&
        check(teleop.handle_key('w', now, exit_requested), "Cartesian input") &&
        check(simulation.step(), "step before Cartesian update") &&
        check(teleop.update(now + std::chrono::milliseconds(10)), "Cartesian update") &&
        check(simulation.step(), "step after Cartesian command") &&
        check(teleop.stop_motion(), "stop motion");

    if (!passed) {
        simulation.shutdown();
        return EXIT_FAILURE;
    }

    RobotState initial;
    passed =
        check(simulation.reset("home"), "reset home") &&
        check(teleop.stop_motion(), "resynchronize after reset") &&
        check(simulation.read_state(initial), "read initial state") &&
        check(teleop.handle_key('R', now, exit_requested), "select right arm") &&
        check(teleop.handle_key('1', now, exit_requested), "select right joint 1") &&
        check(teleop.handle_key('=', now, exit_requested), "right joint positive input");

    auto control_time = now;
    for (int index = 0; passed && index < 200; ++index) {
        control_time += std::chrono::milliseconds(1);
        passed =
            check(simulation.step(), "right arm isolation step") &&
            check(teleop.update(control_time), "right arm isolation update");
    }

    RobotState final_state;
    if (passed) {
        passed = check(simulation.read_state(final_state), "read final state");
    }

    if (passed) {
        const double right_joint_motion =
            final_state.right_arm.joints[0].position -
            initial.right_arm.joints[0].position;
        const double spine_motion =
            std::abs(final_state.spine.position - initial.spine.position);
        const double base_lateral_motion =
            std::abs(final_state.base.pose.position.y - initial.base.pose.position.y);

        std::cout
            << "right_joint_1_motion=" << right_joint_motion
            << " spine_motion=" << spine_motion
            << " base_lateral_motion=" << base_lateral_motion << '\n';

        passed =
            check(right_joint_motion > 0.02, "right joint 1 did not move") &&
            check(spine_motion < 0.02, "spine moved excessively") &&
            check(base_lateral_motion < 0.05, "base moved laterally excessively");
    }

    passed =
        check(teleop.stop_motion(), "final stop motion") &&
        check(simulation.shutdown(), "shutdown") &&
        passed;

    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}