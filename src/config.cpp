#include "configuration.hpp"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include "component_ids.hpp"

namespace mfr3duo_mujoco {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPhysicsPeriod = 0.001;
constexpr double kViewerPeriod = 1.0 / 60.0;

constexpr double degrees(double value) noexcept { return value * kPi / 180.0; }

std::filesystem::path resolve_scene_candidate(const std::filesystem::path& candidate) {
    if (candidate.empty()) return {};

    if (std::filesystem::is_regular_file(candidate)) return candidate;

    const std::filesystem::path package_scene = candidate / "mjcf" / "scene.xml";
    if (std::filesystem::is_regular_file(package_scene)) return package_scene;

    const std::filesystem::path prefix_scene =
        candidate / "share" / "mfr3duo_description" / "mjcf" / "scene.xml";
    if (std::filesystem::is_regular_file(prefix_scene)) return prefix_scene;

    return {};
}

std::filesystem::path resolve_from_prefix_path(const char* value) {
    if (value == nullptr) return {};

    std::string paths(value);
    std::size_t begin = 0;
    while (begin <= paths.size()) {
        const std::size_t end = paths.find(':', begin);
        const std::string path = paths.substr(begin, end - begin);
        const auto resolved = resolve_scene_candidate(path);
        if (!resolved.empty()) return resolved;
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    return {};
}

romujoco::JointInfo make_active_joint(
    romujoco::JointId id, std::string joint_name, std::string actuator_name,
    romujoco::Limit position_limits, double velocity_limit, double effort_limit,
    double stiffness, double damping, bool gravity_compensation = false) {
    romujoco::JointInfo info;
    info.id = id;
    info.joint_name = std::move(joint_name);
    info.actuation = romujoco::JointActuation::Active;
    info.actuator_name = std::move(actuator_name);
    info.default_mode = romujoco::JointMode::Position;
    info.allowed_modes.set(romujoco::JointMode::Hybrid);
    info.allowed_modes.set(romujoco::JointMode::Position);
    info.allowed_modes.set(romujoco::JointMode::Velocity);
    info.allowed_modes.set(romujoco::JointMode::Effort);

    info.hybrid.stiffness = stiffness;
    info.hybrid.damping = damping;
    info.position.stiffness = stiffness;
    info.position.damping = damping;
    info.position.gravity_compensation = gravity_compensation;
    info.velocity.damping = damping;

    info.position_limits = position_limits;
    info.velocity_limits = {-velocity_limit, velocity_limit};
    info.effort_limits = {-effort_limit, effort_limit};
    info.period = kPhysicsPeriod;
    return info;
}

romujoco::JointInfo make_passive_joint(romujoco::JointId id, std::string joint_name) {
    romujoco::JointInfo info;
    info.id = id;
    info.joint_name = std::move(joint_name);
    info.actuation = romujoco::JointActuation::Passive;
    info.default_mode = romujoco::JointMode::None;
    info.period = kPhysicsPeriod;
    return info;
}

void add_fr3_arm(
    romujoco::ComponentConfigList& components, const std::string& prefix,
    romujoco::JointId first_id) {
    struct JointSpec {
        double lower;
        double upper;
        double velocity;
        double effort;
    };

    constexpr JointSpec kSpecs[] = {
        {-2.9007400166666666, 2.9007400166666666, 2.62, 87.0},
        {-1.8360900166666667, 1.8360900166666667, 2.62, 87.0},
        {-2.9007400166666666, 2.9007400166666666, 2.62, 87.0},
        {-3.077020016666667, -0.11693708333333333, 2.62, 87.0},
        {-2.87630335, 2.87630335, 5.26, 12.0},
        {0.43982265, 4.62163335, 4.18, 12.0},
        {-3.05083335, 3.05083335, 5.26, 12.0},
    };

    constexpr double kArmStiffness = 7000.0;
    constexpr double kArmDamping = 50.0;
    for (std::size_t index = 0; index < 7U; ++index) {
        const std::string base = prefix + "fr3v2_1_joint" + std::to_string(index + 1U);
        const JointSpec& spec = kSpecs[index];
        components.emplace_back(make_active_joint(
            first_id + index, base, base + "_motor", {spec.lower, spec.upper},
            spec.velocity, spec.effort, kArmStiffness, kArmDamping));
    }
}

romujoco::GripperInfo make_gripper(
    romujoco::GripperId id, std::string name, const std::string& prefix) {
    romujoco::GripperInfo info;
    info.id = id;
    info.name = std::move(name);
    info.fingers = {{{prefix + "fr3v2_1_finger_joint1", prefix + "fr3v2_1_finger_motor"},
                     {prefix + "fr3v2_1_finger_joint2", ""}}};
    info.period = kPhysicsPeriod;
    info.control.stiffness = 200.0;
    info.control.damping = 8.0;
    info.width_limits = {0.0, 0.08};
    info.velocity_limits = {0.0, 0.20};
    info.effort_limits = {0.0, 100.0};
    info.stall.width_tolerance = 0.001;
    info.stall.velocity_threshold = 0.001;
    info.stall.effort_ratio = 0.9;
    info.stall.timeout = 0.25;
    return info;
}

romujoco::SwerveMobileBaseInfo make_mobile_base() {
    romujoco::SwerveMobileBaseInfo info;
    info.common.id = detail::component_ids::mobile_base::kTmr;
    info.common.name = "tmr";
    info.common.base_body_name = "base_link";
    info.common.execution_mode = romujoco::MobileBaseExecutionMode::Dynamic;
    info.common.period = kPhysicsPeriod;
    info.modules = {
        {"front", 0.3, -0.2, 0.05, "tmrv0_2_joint_0", "tmrv0_2_joint_0_position",
         "tmrv0_2_joint_1", "tmrv0_2_joint_1_velocity"},
        {"rear", -0.3, 0.2, 0.05, "tmrv0_2_joint_2", "tmrv0_2_joint_2_position",
         "tmrv0_2_joint_3", "tmrv0_2_joint_3_velocity"},
    };
    return info;
}

romujoco::ImuInfo make_imu() {
    romujoco::ImuInfo info;
    info.id = detail::component_ids::imu::kBase;
    info.name = "imu";
    info.frame_id = "imu_sensor_frame";
    info.framequat_sensor_name = "imu_orientation";
    info.gyro_sensor_name = "imu_angular_velocity";
    info.accelerometer_sensor_name = "imu_linear_acceleration";
    info.period = kPhysicsPeriod;
    return info;
}

romujoco::LidarInfo make_lidar(
    romujoco::LidarId id, std::string name, std::string frame_id, std::string site_name,
    double period) {
    romujoco::LidarInfo info;
    info.id = id;
    info.name = std::move(name);
    info.frame_id = std::move(frame_id);
    info.site_name = std::move(site_name);
    info.output = romujoco::LidarOutput::LaserScan;
    info.period = period;
    info.azimuth_start = degrees(-47.5);
    info.azimuth_increment = degrees(0.17);
    info.azimuth_samples = 1618;
    info.channels = {{0.0, 0.0}};
    info.range_min = 0.05;
    info.range_max = 40.0;
    info.geom_group_mask = (1U << 1U) | (1U << 3U);
    info.exclude_parent_body = true;
    return info;
}

romujoco::CameraConfig make_camera(
    romujoco::CameraId id, std::string name, std::string frame_id,
    std::string optical_frame_id, std::string camera_name, bool rgb, bool depth,
    const SimulationOptions& options) {
    romujoco::CameraConfig info;
    info.id = id;
    info.name = std::move(name);
    info.frame_id = std::move(frame_id);
    info.optical_frame_id = std::move(optical_frame_id);
    info.camera_name = std::move(camera_name);
    info.period = options.camera_period;
    info.width = options.camera_width;
    info.height = options.camera_height;
    info.enable_rgb = rgb;
    info.enable_depth = depth;
    return info;
}

void add_cameras(
    romujoco::ComponentConfigList& components, const SimulationOptions& options) {
    using namespace detail::component_ids;
    components.emplace_back(make_camera(
        camera::kFrontColor, "front_color", "camera_front_color_frame",
        "camera_front_color_optical_frame", "camera_front_color", true, false, options));
    components.emplace_back(make_camera(
        camera::kFrontDepth, "front_depth", "camera_front_depth_frame",
        "camera_front_depth_optical_frame", "camera_front_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kRearColor, "rear_color", "camera_rear_color_frame",
        "camera_rear_color_optical_frame", "camera_rear_color", true, false, options));
    components.emplace_back(make_camera(
        camera::kRearDepth, "rear_depth", "camera_rear_depth_frame",
        "camera_rear_depth_optical_frame", "camera_rear_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kRightColor, "right_color", "camera_right_color_frame",
        "camera_right_color_optical_frame", "camera_right_color", true, false, options));
    components.emplace_back(make_camera(
        camera::kRightDepth, "right_depth", "camera_right_depth_frame",
        "camera_right_depth_optical_frame", "camera_right_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kLeftColor, "left_color", "camera_left_color_frame",
        "camera_left_color_optical_frame", "camera_left_color", true, false, options));
    components.emplace_back(make_camera(
        camera::kLeftDepth, "left_depth", "camera_left_depth_frame",
        "camera_left_depth_optical_frame", "camera_left_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kLeftWristColor, "left_wrist_color", "left_d435_link",
        "left_d435_color_optical_frame", "d435_left_rgb", true, false, options));
    components.emplace_back(make_camera(
        camera::kLeftWristDepth, "left_wrist_depth", "left_d435_link",
        "left_d435_depth_optical_frame", "d435_left_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kRightWristColor, "right_wrist_color", "right_d435_link",
        "right_d435_color_optical_frame", "d435_right_rgb", true, false, options));
    components.emplace_back(make_camera(
        camera::kRightWristDepth, "right_wrist_depth", "right_d435_link",
        "right_d435_depth_optical_frame", "d435_right_depth", false, true, options));
    components.emplace_back(make_camera(
        camera::kHeadZedLeft, "head_zed_left", "head_zed_left_camera_frame",
        "head_zed_left_camera_optical_frame", "head_zed_left", true, false, options));
    components.emplace_back(make_camera(
        camera::kHeadZedRight, "head_zed_right", "head_zed_right_camera_frame",
        "head_zed_right_camera_optical_frame", "head_zed_right", true, false, options));
}

void validate_options(const std::string& model_path, const SimulationOptions& options) {
    if (model_path.empty()) {
        throw std::invalid_argument("MFR3Duo model path must not be empty");
    }
    if (options.cameras_enabled &&
        (options.camera_width <= 0 || options.camera_height <= 0 ||
         !std::isfinite(options.camera_period) || options.camera_period <= 0.0)) {
        throw std::invalid_argument("MFR3Duo camera configuration is invalid");
    }
    if (options.lidars_enabled &&
        (!std::isfinite(options.lidar_period) || options.lidar_period <= 0.0)) {
        throw std::invalid_argument("MFR3Duo lidar period must be finite and positive");
    }
}

}  // namespace

std::string scene_path() {
    if (const char* value = std::getenv("MFR3DUO_DESCRIPTION_PATH")) {
        const auto resolved = resolve_scene_candidate(value);
        if (!resolved.empty()) return resolved.string();
    }

    const auto from_prefix = resolve_from_prefix_path(std::getenv("CMAKE_PREFIX_PATH"));
    if (!from_prefix.empty()) return from_prefix.string();

#ifdef MFR3DUO_MUJOCO_DEFAULT_DESCRIPTION_SHARE_DIR
    const auto built_path =
        resolve_scene_candidate(MFR3DUO_MUJOCO_DEFAULT_DESCRIPTION_SHARE_DIR);
    if (!built_path.empty()) return built_path.string();
#endif

    throw std::runtime_error(
        "unable to locate mfr3duo_description/mjcf/scene.xml; set "
        "MFR3DUO_DESCRIPTION_PATH, expose its prefix through CMAKE_PREFIX_PATH, "
        "or initialize Simulation with an explicit model path");
}

namespace detail {

romujoco::SimulationConfig make_simulation_config(
    const std::string& model_path, const SimulationOptions& options) {
    validate_options(model_path, options);

    romujoco::SimulationConfig config;
    config.model.model_path = model_path;
    config.model.initial_keyframe = options.initial_keyframe;
    config.scheduler.physics_period = kPhysicsPeriod;
    config.scheduler.viewer_period = kViewerPeriod;
    config.viewer_enabled = options.viewer_enabled;

    config.components.emplace_back(make_active_joint(
        component_ids::joint::kSpine, "franka_spine_vertical_joint",
        "franka_spine_motor", {0.0, 0.85}, 0.1, 600.0, 5000.0, 200.0, true));

    add_fr3_arm(config.components, "left_", component_ids::joint::kLeftArm.front());
    add_fr3_arm(config.components, "right_", component_ids::joint::kRightArm.front());

    config.components.emplace_back(make_passive_joint(
        component_ids::joint::kCasterFrontLeftSteering,
        "caster_front_left_steering_joint"));
    config.components.emplace_back(make_passive_joint(
        component_ids::joint::kCasterFrontLeftWheel, "caster_front_left_joint"));
    config.components.emplace_back(make_passive_joint(
        component_ids::joint::kRockerArm, "rocker_arm_joint"));
    config.components.emplace_back(make_passive_joint(
        component_ids::joint::kCasterRearRightSteering,
        "caster_rear_right_steering_joint"));
    config.components.emplace_back(make_passive_joint(
        component_ids::joint::kCasterRearRightWheel, "caster_rear_right_joint"));

    config.components.emplace_back(
        make_gripper(component_ids::gripper::kLeft, "left_gripper", "left_"));
    config.components.emplace_back(
        make_gripper(component_ids::gripper::kRight, "right_gripper", "right_"));
    config.components.emplace_back(make_mobile_base());

    if (options.imu_enabled) config.components.emplace_back(make_imu());
    if (options.lidars_enabled) {
        config.components.emplace_back(make_lidar(
            component_ids::lidar::kFront, "lidar_front", "lidar_front_scan_frame",
            "lidar_front_scan_frame", options.lidar_period));
        config.components.emplace_back(make_lidar(
            component_ids::lidar::kRear, "lidar_rear", "lidar_rear_scan_frame",
            "lidar_rear_scan_frame", options.lidar_period));
    }
    if (options.cameras_enabled) add_cameras(config.components, options);

    return config;
}

romujoco::SimulationConfig make_simulation_config(const SimulationOptions& options) {
    return make_simulation_config(scene_path(), options);
}

}  // namespace detail
}  // namespace mfr3duo_mujoco
