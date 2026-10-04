#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "mfr3duo_mujoco/simulation.hpp"

int main() {
    using namespace mfr3duo_mujoco;
    SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;
    options.lidars_enabled = false;
    options.imu_enabled = false;
    Simulation simulation;
    if (!simulation.initialize(options) || !simulation.step(2000)) return EXIT_FAILURE;
    double largest = 0;
    // Every trajectory starts from measured positions. Replacing a static hold
    // with those positions must not create another gravity sag on each handoff.
    for (int handoff = 0; handoff < 4; ++handoff) {
        ArmState left, right;
        if (!simulation.read_state(Arm::Left, left) || !simulation.read_state(Arm::Right, right))
            return EXIT_FAILURE;
        ArmCommand lc, rc;
        for (std::size_t i = 0; i < kArmJointCount; ++i) {
            lc.joints[i].position = left.joints[i].position;
            rc.joints[i].position = right.joints[i].position;
        }
        if (!simulation.write_command(Arm::Left, lc) || !simulation.write_command(Arm::Right, rc))
            return EXIT_FAILURE;
        for (int step = 0; step < 1500; ++step) {
            ArmState actual_left, actual_right;
            if (!simulation.step() || !simulation.read_state(Arm::Left, actual_left) ||
                !simulation.read_state(Arm::Right, actual_right))
                return EXIT_FAILURE;
            for (std::size_t i = 0; i < kArmJointCount; ++i) {
                const double error = std::max(
                    std::abs(actual_left.joints[i].position - left.joints[i].position),
                    std::abs(actual_right.joints[i].position - right.joints[i].position));
                if (!std::isfinite(error) || error > .0005) {
                    std::cerr << "Measured-start handoff sag: joint=" << i << " error=" << error
                              << '\n';
                    return EXIT_FAILURE;
                }
                largest = std::max(largest, error);
            }
        }
    }
    std::cout << "ARM_MEASURED_HOLD_PASS full transient maximum drift=" << largest << '\n';
    return simulation.shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
