#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mfr3duo_mujoco {

/**
 * @brief Canonical MFR3Duo 2D LiDAR devices.
 */
enum class Lidar : std::uint8_t {
    Front,
    Rear,
};

/**
 * @brief One nanoScan3 scan sample.
 */
struct LaserScan {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    std::string frame_id;
    float angle_min{0.0F};
    float angle_max{0.0F};
    float angle_increment{0.0F};
    float time_increment{0.0F};
    float scan_time{0.0F};
    float range_min{0.0F};
    float range_max{0.0F};
    std::vector<float> ranges;
    std::vector<float> intensities;
};

}  // namespace mfr3duo_mujoco
