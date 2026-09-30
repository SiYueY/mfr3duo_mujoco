#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <variant>

#include "component_ids.hpp"
#include "configuration.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

template <typename T>
std::size_t count(const romujoco::ComponentConfigList& components) {
    std::size_t result = 0;
    for (const auto& component : components) {
        if (std::holds_alternative<T>(component)) ++result;
    }
    return result;
}

const romujoco::JointInfo* find_joint(
    const romujoco::ComponentConfigList& components, romujoco::JointId id) {
    for (const auto& component : components) {
        const auto* joint = std::get_if<romujoco::JointInfo>(&component);
        if (joint != nullptr && joint->id == id) return joint;
    }
    return nullptr;
}

}  // namespace

int main() {
    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;
    const auto config = mfr3duo_mujoco::detail::make_simulation_config(
        "/tmp/mfr3duo_description/mjcf/scene.xml", options);

    const bool passed =
        check(config.model.model_path == "/tmp/mfr3duo_description/mjcf/scene.xml", "model path") &&
        check(config.model.initial_keyframe == "home", "initial keyframe") &&
        check(config.scheduler.physics_period == 0.001, "physics period") &&
        check(!config.viewer_enabled, "viewer option") &&
        check(count<romujoco::JointInfo>(config.components) == 24U, "joint count") &&
        check(count<romujoco::GripperInfo>(config.components) == 2U, "gripper count") &&
        check(
            count<romujoco::SwerveMobileBaseInfo>(config.components) == 0U,
            "mobile base count") &&
        check(count<romujoco::ImuInfo>(config.components) == 1U, "imu count") &&
        check(count<romujoco::LidarInfo>(config.components) == 2U, "lidar count") &&
        check(count<romujoco::CameraConfig>(config.components) == 14U, "camera count") &&
        check(config.components.size() == 43U, "total component count");

    namespace tmr = mfr3duo_mujoco::detail::component_ids::joint::tmr;
    for (const auto id : {tmr::kFrontSteering, tmr::kFrontDrive,
                          tmr::kRearSteering, tmr::kRearDrive}) {
        const auto* joint = find_joint(config.components, id);
        if (!check(joint != nullptr, "missing TMR joint")) return EXIT_FAILURE;
        const auto mode = (id == tmr::kFrontSteering || id == tmr::kRearSteering)
                              ? romujoco::JointMode::Position
                              : romujoco::JointMode::Velocity;
        if (!check(joint->default_mode == mode &&
                       joint->allowed_modes.contains(mode) &&
                       !joint->allowed_modes.contains(romujoco::JointMode::Hybrid) &&
                       joint->actuator_name == joint->joint_name + "_motor" &&
                       joint->effort_limits.min == -500.0 &&
                       joint->effort_limits.max == 500.0,
                   "TMR motor contract")) return EXIT_FAILURE;
    }
    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
