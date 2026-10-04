#pragma once

#include <cstdint>

#include "mfr3duo_mujoco/data/math.hpp"

namespace mfr3duo_mujoco {

/** @brief Read-only world pose sampled by the owning simulation instance. */
struct BasePoseState {
    std::uint64_t sequence{0};
    double timestamp{0.0};
    Pose pose;
};

}  // namespace mfr3duo_mujoco
