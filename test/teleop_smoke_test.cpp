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
    const bool passed =
        check(teleop.handle_key('L', now, exit_requested), "select left arm") &&
        check(!exit_requested, "unexpected exit request") &&
        check(teleop.handle_key('M', now, exit_requested), "select Cartesian mode") &&
        check(teleop.handle_key('w', now, exit_requested), "Cartesian input") &&
        check(teleop.update(now + std::chrono::milliseconds(10), 0.01), "Cartesian update") &&
        check(simulation.step(), "step after Cartesian command") &&
        check(teleop.stop_motion(), "stop motion") &&
        check(simulation.shutdown(), "shutdown");

    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
