#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "mfr3duo_mujoco/data/math.hpp"

namespace mfr3duo_mujoco {

/**
 * @brief State reported by the base IMU.
 */
struct ImuState {
    std::uint64_t sequence{0};
    double timestamp{0.0};
    std::string frame_id;
    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
};

}  // namespace mfr3duo_mujoco
