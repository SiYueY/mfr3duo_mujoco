#include <cstdlib>

#include "mfr3duo_mujoco/simulation.hpp"

int main() {
    mfr3duo_mujoco::Simulation simulation;
    return simulation.status() == mfr3duo_mujoco::SimulationStatus::Uninitialized
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
