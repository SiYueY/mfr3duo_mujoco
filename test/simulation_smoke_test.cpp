#include <cstdlib>
#include <exception>
#include <iostream>

#include "mfr3duo_mujoco/config.hpp"
#include "romujoco/simulation.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

template <typename States>
bool size_is(const States& states, std::size_t expected) {
    return states != nullptr && states->size() == expected;
}

}  // namespace

int main() {
    try {
        mfr3duo_mujoco::SimulationOptions options;
        options.viewer_enabled = false;
        options.cameras_enabled = false;

        romujoco::Simulation simulation;
        if (!check(
                simulation.initialize(mfr3duo_mujoco::make_simulation_config(options)),
                "failed to initialize the real MFR3Duo model")) {
            return EXIT_FAILURE;
        }
        if (!check(simulation.step(5), "failed to step the MFR3Duo simulation")) {
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        romujoco::RobotState state;
        const bool passed =
            check(simulation.read_state(state), "failed to read robot state") &&
            check(size_is(state.joints, 20U), "unexpected joint state count") &&
            check(size_is(state.grippers, 2U), "unexpected gripper state count") &&
            check(size_is(state.mobile_bases, 1U), "unexpected mobile-base state count") &&
            check(size_is(state.imus, 1U), "unexpected IMU state count") &&
            check(size_is(state.laser_scans, 2U), "unexpected lidar state count") &&
            check(state.cameras == nullptr || state.cameras->empty(), "cameras were not disabled") &&
            check(simulation.shutdown(), "failed to shut down the simulation");
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
