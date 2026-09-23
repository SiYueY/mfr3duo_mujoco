#pragma once

#include <string>

#include "romujoco/config/simulation_config.hpp"

namespace mfr3duo_mujoco {

struct SimulationOptions {
    bool viewer_enabled{true};
    bool cameras_enabled{true};
    bool lidars_enabled{true};
    bool imu_enabled{true};

    std::string initial_keyframe{"home"};

    int camera_width{1280};
    int camera_height{720};
    double camera_period{1.0 / 30.0};
    double lidar_period{0.04};
};

std::string description_share_directory();
std::string scene_path();

romujoco::SimulationConfig make_simulation_config(
    const std::string& model_path, const SimulationOptions& options = {});

romujoco::SimulationConfig make_simulation_config(const SimulationOptions& options = {});

}  // namespace mfr3duo_mujoco
