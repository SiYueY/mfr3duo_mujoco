#pragma once

#include <string>

namespace mfr3duo_mujoco {

/**
 * @brief Runtime options for the canonical Mobile FR3 Duo simulation.
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
 * @brief Resolve the canonical MFR3Duo scene.
 *
 * Resolution checks MFR3DUO_DESCRIPTION_PATH, CMAKE_PREFIX_PATH and the
 * description path discovered when this library was built.
 *
 * @throws std::runtime_error when no canonical scene can be resolved.
 */
std::string scene_path();

}  // namespace mfr3duo_mujoco
