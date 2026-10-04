#pragma once
#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include "mfr3duo_mujoco/data/math.hpp"
namespace mfr3duo_mujoco {
struct GraspObservation {
    bool valid{false};
    bool object_visible{false};
    bool left_finger_contact{false};
    bool right_finger_contact{false};
    std::uint64_t sequence{0};
    double timestamp{0};
    Pose object_pose;
    Pose tool_pose;
    std::string diagnostic;
};
}  // namespace mfr3duo_mujoco
