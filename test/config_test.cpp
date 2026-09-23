#include <cstdlib>
#include <iostream>
#include <variant>

#include "mfr3duo_mujoco/config.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

template <typename T>
std::size_t count(const romujoco::ComponentConfigList& components) {
    std::size_t result = 0;
    for (const auto& component : components)
        if (std::holds_alternative<T>(component)) ++result;
    return result;
}

}  // namespace

int main() {
    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;
    const auto config =
        mfr3duo_mujoco::make_simulation_config("/tmp/mfr3duo_description/mjcf/scene.xml", options);

    const bool passed =
        check(config.model.model_path == "/tmp/mfr3duo_description/mjcf/scene.xml", "model path") &&
        check(config.model.initial_keyframe == "home", "initial keyframe") &&
        check(config.scheduler.physics_period == 0.001, "physics period") &&
        check(!config.viewer_enabled, "viewer option") &&
        check(count<romujoco::JointInfo>(config.components) == 20U, "joint count") &&
        check(count<romujoco::GripperInfo>(config.components) == 2U, "gripper count") &&
        check(count<romujoco::SwerveMobileBaseInfo>(config.components) == 1U, "mobile base count") &&
        check(count<romujoco::ImuInfo>(config.components) == 1U, "imu count") &&
        check(count<romujoco::LidarInfo>(config.components) == 2U, "lidar count") &&
        check(count<romujoco::CameraConfig>(config.components) == 14U, "camera count") &&
        check(config.components.size() == 40U, "total component count");

    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
