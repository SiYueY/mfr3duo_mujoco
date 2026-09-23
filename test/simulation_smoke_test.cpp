#include <cstdlib>
#include <exception>
#include <iostream>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main() {
    try {
        mfr3duo_mujoco::SimulationOptions options;
        options.viewer_enabled = false;
        options.cameras_enabled = false;

        mfr3duo_mujoco::Simulation simulation;
        if (!check(
                simulation.initialize(options),
                "failed to initialize the real MFR3Duo model")) {
            return EXIT_FAILURE;
        }

        // Advance beyond the 25 Hz LiDAR period so the test does not depend on
        // whether an initial scan is published during component initialization.
        if (!check(simulation.step(50), "failed to step the MFR3Duo simulation")) {
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        mfr3duo_mujoco::RobotState state;
        mfr3duo_mujoco::ImuState imu;
        mfr3duo_mujoco::LaserScan front_scan;
        mfr3duo_mujoco::LaserScan rear_scan;

        const bool passed =
            check(simulation.read_state(state), "failed to read robot state") &&
            check(simulation.read_imu_state(imu), "failed to read IMU state") &&
            check(
                simulation.read_lidar(mfr3duo_mujoco::Lidar::Front, front_scan),
                "failed to read front LiDAR") &&
            check(
                simulation.read_lidar(mfr3duo_mujoco::Lidar::Rear, rear_scan),
                "failed to read rear LiDAR") &&
            check(!front_scan.ranges.empty(), "front LiDAR scan is empty") &&
            check(!rear_scan.ranges.empty(), "rear LiDAR scan is empty") &&
            check(simulation.start(), "failed to start continuous simulation") &&
            check(!simulation.step(), "step unexpectedly succeeded while running") &&
            check(simulation.pause(), "failed to pause continuous simulation") &&
            check(!simulation.step(), "step unexpectedly succeeded while paused") &&
            check(simulation.resume(), "failed to resume continuous simulation") &&
            check(simulation.stop(), "failed to stop continuous simulation") &&
            check(simulation.step(), "step failed after continuous simulation stopped") &&
            check(simulation.shutdown(), "failed to shut down the simulation");

        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}