#include <cstdlib>
#include <iostream>

#include "mfr3duo_mujoco/simulation.hpp"

int main() {
    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;

    mfr3duo_mujoco::Simulation simulation;
    if (!simulation.initialize(options)) {
        std::cerr << "failed to initialize simulation\n";
        return EXIT_FAILURE;
    }

    mfr3duo_mujoco::ArmState left_state;
    if (!simulation.read_arm_state(mfr3duo_mujoco::Arm::Left, left_state)) {
        return EXIT_FAILURE;
    }

    mfr3duo_mujoco::ArmCommand left_command;
    for (std::size_t index = 0; index < left_command.joints.size(); ++index) {
        left_command.joints[index].mode = mfr3duo_mujoco::JointControlMode::Position;
        left_command.joints[index].position = left_state.joints[index].position;
    }
    left_command.joints[0].position += 0.2;

    if (!simulation.write_arm_command(mfr3duo_mujoco::Arm::Left, left_command)) {
        return EXIT_FAILURE;
    }

    mfr3duo_mujoco::BaseCommand base_command;
    base_command.linear_x = 0.2;
    if (!simulation.write_base_command(base_command)) {
        return EXIT_FAILURE;
    }

    if (!simulation.step(1000)) {
        return EXIT_FAILURE;
    }

    mfr3duo_mujoco::RobotState state;
    if (!simulation.read_state(state)) {
        return EXIT_FAILURE;
    }

    std::cout << "simulation time: " << state.simulation_time << '\n';
    return simulation.shutdown() ? EXIT_SUCCESS : EXIT_FAILURE;
}
