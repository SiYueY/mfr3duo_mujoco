#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

bool check_response(mfr3duo_mujoco::Simulation& simulation, double target) {
    mfr3duo_mujoco::SpineState state;
    if (!simulation.read_state(state)) return false;
    const double lower = std::min(state.position, target) - 0.002;
    const double upper = std::max(state.position, target) + 0.002;
    mfr3duo_mujoco::SpineCommand command;
    command.position = target;
    if (!simulation.write_command(command)) return false;

    // Measure the entire transient, rather than passing at the first crossing
    // of the target. A lightly damped lift can reach the target and then swing.
    const double start = simulation.time();
    while (simulation.time() - start < 3.0) {
        if (!simulation.step() || !simulation.read_state(state)) return false;
        if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
            state.position < lower || state.position > upper) {
            std::cerr << "lift overshoot: target=" << target
                      << ", actual=" << state.position << '\n';
            return false;
        }
        if (simulation.time() - start >= 2.0 &&
            (std::abs(state.position - target) > 0.001 ||
             std::abs(state.velocity) > 0.005)) {
            std::cerr << "lift did not settle: target=" << target
                      << ", actual=" << state.position
                      << ", velocity=" << state.velocity << '\n';
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;
    options.lidars_enabled = false;
    options.imu_enabled = false;
    mfr3duo_mujoco::Simulation simulation;
    if (!simulation.initialize(options) || !simulation.step(1000)) {
        return EXIT_FAILURE;
    }
    for (const double target : {0.24, 0.20, 0.40, 0.20}) {
        if (!check_response(simulation, target)) return EXIT_FAILURE;
    }
    return simulation.shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
