#pragma once

#include <string>

#include "mfr3duo_mujoco/config.hpp"
#include "romujoco/config/simulation_config.hpp"

namespace mfr3duo_mujoco::detail {

romujoco::SimulationConfig make_simulation_config(
    const std::string& model_path, const SimulationOptions& options);

romujoco::SimulationConfig make_simulation_config(const SimulationOptions& options);

}  // namespace mfr3duo_mujoco::detail
