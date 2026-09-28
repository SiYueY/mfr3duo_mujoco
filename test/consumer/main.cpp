#include <cstdlib>

#include "mfr3duo_mujoco/data/robot.hpp"
#include "mfr3duo_mujoco/simulation.hpp"

int main() {
    mfr3duo_mujoco::RobotCommand command;
    mfr3duo_mujoco::RobotState state;
    mfr3duo_mujoco::Simulation simulation;
    if (command.left_arm.joints.size() != 7 || state.right_arm.joints.size() != 7) {
        return EXIT_FAILURE;
    }
    return simulation.status() == mfr3duo_mujoco::SimulationStatus::Uninitialized
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
