#pragma once

#include <string>

#include "romujoco/config/simulation_config.hpp"

namespace mfr3duo_mujoco {

/**
 * @brief Runtime options for the canonical Mobile FR3 Duo simulation assembly.
 */
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

/**
 * @brief Return the installed share directory of mfr3duo_description.
 * @throws ament_index_cpp::PackageNotFoundError when the description package is unavailable.
 */
std::string description_share_directory();

/**
 * @brief Resolve the canonical installed MuJoCo scene.
 * @throws std::runtime_error when the installed scene file is missing.
 */
std::string scene_path();

/**
 * @brief Build the complete MFR3Duo romujoco configuration for an explicit MJCF path.
 * @throws std::invalid_argument when runtime options are invalid.
 */
romujoco::SimulationConfig make_simulation_config(
    const std::string& model_path, const SimulationOptions& options = {});

/**
 * @brief Build the complete MFR3Duo configuration using the installed description package.
 */
romujoco::SimulationConfig make_simulation_config(const SimulationOptions& options = {});

}  // namespace mfr3duo_mujoco
