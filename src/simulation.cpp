#include "mfr3duo_mujoco/simulation.hpp"

#include <utility>

#include "component_ids.hpp"
#include "configuration.hpp"
#include "romujoco/simulation.hpp"

namespace mfr3duo_mujoco {
namespace {

romujoco::JointMode to_romujoco_mode(JointControlMode mode) {
    switch (mode) {
        case JointControlMode::Hybrid:
            return romujoco::JointMode::Hybrid;
        case JointControlMode::Position:
            return romujoco::JointMode::Position;
        case JointControlMode::Velocity:
            return romujoco::JointMode::Velocity;
        case JointControlMode::Effort:
            return romujoco::JointMode::Effort;
    }
    return romujoco::JointMode::None;
}

JointControlMode from_romujoco_mode(std::uint8_t mode) {
    switch (static_cast<romujoco::JointMode>(mode)) {
        case romujoco::JointMode::Hybrid:
            return JointControlMode::Hybrid;
        case romujoco::JointMode::Velocity:
            return JointControlMode::Velocity;
        case romujoco::JointMode::Effort:
            return JointControlMode::Effort;
        case romujoco::JointMode::Position:
        case romujoco::JointMode::None:
            return JointControlMode::Position;
    }
    return JointControlMode::Position;
}

romujoco::JointCommand make_joint_command(
    romujoco::JointId id, const JointCommand& command) {
    romujoco::JointCommand result;
    result.id = id;
    result.mode = static_cast<std::uint8_t>(to_romujoco_mode(command.mode));
    result.position = command.position;
    result.velocity = command.velocity;
    result.effort = command.effort;
    result.stiffness = command.stiffness;
    result.damping = command.damping;
    return result;
}

const std::array<romujoco::JointId, kArmJointCount>& arm_ids(Arm arm) {
    return arm == Arm::Left ? detail::component_ids::joint::kLeftArm
                            : detail::component_ids::joint::kRightArm;
}

romujoco::GripperId gripper_id(Gripper gripper) {
    return gripper == Gripper::Left ? detail::component_ids::gripper::kLeft
                                    : detail::component_ids::gripper::kRight;
}

romujoco::CameraId camera_id(Camera value) {
    switch (value) {
        case Camera::FrontColor:
            return detail::component_ids::camera::kFrontColor;
        case Camera::FrontDepth:
            return detail::component_ids::camera::kFrontDepth;
        case Camera::RearColor:
            return detail::component_ids::camera::kRearColor;
        case Camera::RearDepth:
            return detail::component_ids::camera::kRearDepth;
        case Camera::RightColor:
            return detail::component_ids::camera::kRightColor;
        case Camera::RightDepth:
            return detail::component_ids::camera::kRightDepth;
        case Camera::LeftColor:
            return detail::component_ids::camera::kLeftColor;
        case Camera::LeftDepth:
            return detail::component_ids::camera::kLeftDepth;
        case Camera::LeftWristColor:
            return detail::component_ids::camera::kLeftWristColor;
        case Camera::LeftWristDepth:
            return detail::component_ids::camera::kLeftWristDepth;
        case Camera::RightWristColor:
            return detail::component_ids::camera::kRightWristColor;
        case Camera::RightWristDepth:
            return detail::component_ids::camera::kRightWristDepth;
        case Camera::HeadZedLeft:
            return detail::component_ids::camera::kHeadZedLeft;
        case Camera::HeadZedRight:
            return detail::component_ids::camera::kHeadZedRight;
    }
    return detail::component_ids::camera::kFrontColor;
}

romujoco::LidarId lidar_id(Lidar lidar) {
    return lidar == Lidar::Front ? detail::component_ids::lidar::kFront
                                 : detail::component_ids::lidar::kRear;
}

template <typename State>
const State* find_state(
    const romujoco::StateSnapshots<State>& states, std::size_t id) {
    if (states == nullptr) return nullptr;
    for (const auto& state : *states) {
        if (state != nullptr && state->id == id) return state.get();
    }
    return nullptr;
}

void copy_joint_state(const romujoco::JointState& source, JointState& target) {
    target.timestamp = source.timestamp;
    target.mode = from_romujoco_mode(source.mode);
    target.position = source.position;
    target.velocity = source.velocity;
    target.effort = source.effort;
}

bool copy_arm_state(
    const romujoco::JointStates& states,
    const std::array<romujoco::JointId, kArmJointCount>& ids,
    ArmState& target) {
    for (std::size_t index = 0; index < ids.size(); ++index) {
        const auto* source = find_state(states, ids[index]);
        if (source == nullptr) return false;
        copy_joint_state(*source, target.joints[index]);
    }
    return true;
}

void copy_gripper_state(const romujoco::GripperState& source, GripperState& target) {
    target.timestamp = source.timestamp;
    target.width = source.width;
    target.velocity = source.velocity;
    target.effort = source.effort;
    target.stalled = source.stalled;
}

void copy_base_state(const romujoco::MobileBaseState& source, BaseState& target) {
    target.timestamp = source.timestamp;
    target.pose.position = {
        source.pose.position[0], source.pose.position[1], source.pose.position[2]};
    target.pose.orientation = {
        source.pose.orientation[1],
        source.pose.orientation[2],
        source.pose.orientation[3],
        source.pose.orientation[0]};
    target.twist.linear = {
        source.twist.linear[0], source.twist.linear[1], source.twist.linear[2]};
    target.twist.angular = {
        source.twist.angular[0], source.twist.angular[1], source.twist.angular[2]};
}

Image copy_image(const romujoco::Image& source) {
    Image target;
    target.timestamp = source.timestamp;
    target.frame_id = source.frame_id;
    target.height = source.height;
    target.width = source.width;
    target.encoding = source.encoding;
    target.is_bigendian = source.is_bigendian;
    target.step = source.step;
    target.data = source.data;
    return target;
}

CameraInfo copy_camera_info(const romujoco::CameraInfo& source) {
    CameraInfo target;
    target.height = source.height;
    target.width = source.width;
    target.distortion_model = source.distortion_model;
    target.d = source.d;
    target.k = source.k;
    target.r = source.r;
    target.p = source.p;
    target.binning_x = source.binning_x;
    target.binning_y = source.binning_y;
    return target;
}

SimulationStatus from_romujoco_status(romujoco::SimulationStatus status) {
    switch (status) {
        case romujoco::SimulationStatus::Uninitialized:
            return SimulationStatus::Uninitialized;
        case romujoco::SimulationStatus::Stopped:
            return SimulationStatus::Stopped;
        case romujoco::SimulationStatus::Running:
            return SimulationStatus::Running;
        case romujoco::SimulationStatus::Paused:
            return SimulationStatus::Paused;
        case romujoco::SimulationStatus::Stopping:
            return SimulationStatus::Stopping;
        case romujoco::SimulationStatus::Error:
            return SimulationStatus::Error;
    }
    return SimulationStatus::Error;
}

}  // namespace

class Simulation::Impl {
public:
    romujoco::Simulation simulation;
};

Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}

Simulation::~Simulation() = default;

bool Simulation::initialize(const SimulationOptions& options) {
    return impl_->simulation.initialize(detail::make_simulation_config(options));
}

bool Simulation::initialize(
    const std::string& model_path, const SimulationOptions& options) {
    return impl_->simulation.initialize(detail::make_simulation_config(model_path, options));
}

bool Simulation::shutdown() { return impl_->simulation.shutdown(); }

bool Simulation::start() { return impl_->simulation.start(); }

bool Simulation::stop() { return impl_->simulation.stop(); }

bool Simulation::pause() { return impl_->simulation.pause(); }

bool Simulation::resume() { return impl_->simulation.resume(); }

bool Simulation::reset() { return impl_->simulation.reset(); }

bool Simulation::reset(const std::string& keyframe_name) {
    return impl_->simulation.reset(keyframe_name);
}

bool Simulation::write_arm_command(Arm arm, const ArmCommand& command) {
    const auto& ids = arm_ids(arm);
    romujoco::JointCommands commands;
    commands.reserve(kArmJointCount);
    for (std::size_t index = 0; index < ids.size(); ++index) {
        commands.emplace_back(make_joint_command(ids[index], command.joints[index]));
    }
    return impl_->simulation.write_commands(commands);
}

bool Simulation::write_gripper_command(
    Gripper gripper, const GripperCommand& command) {
    romujoco::GripperCommand result;
    result.id = gripper_id(gripper);
    result.width = command.width;
    result.velocity = command.velocity;
    result.effort = command.effort;
    return impl_->simulation.write_command(result);
}

bool Simulation::write_spine_command(const SpineCommand& command) {
    return impl_->simulation.write_command(
        make_joint_command(detail::component_ids::joint::kSpine, command));
}

bool Simulation::write_base_command(const BaseCommand& command) {
    romujoco::MobileBaseCommand result;
    result.id = detail::component_ids::mobile_base::kTmr;
    result.velocity.linear_x = command.linear_x;
    result.velocity.linear_y = command.linear_y;
    result.velocity.angular_z = command.angular_z;
    return impl_->simulation.write_command(result);
}

bool Simulation::read_state(RobotState& state) const {
    romujoco::RobotState source;
    if (!impl_->simulation.read_state(source)) return false;

    const auto* spine =
        find_state(source.joints, detail::component_ids::joint::kSpine);
    const auto* left_gripper =
        find_state(source.grippers, detail::component_ids::gripper::kLeft);
    const auto* right_gripper =
        find_state(source.grippers, detail::component_ids::gripper::kRight);
    const auto* base =
        find_state(source.mobile_bases, detail::component_ids::mobile_base::kTmr);
    if (spine == nullptr || left_gripper == nullptr || right_gripper == nullptr ||
        base == nullptr) {
        return false;
    }

    RobotState result;
    result.sequence = source.sequence;
    result.timestamp = source.timestamp;
    result.simulation_time = source.simulation_time;
    result.step = source.step;
    copy_joint_state(*spine, result.spine);
    if (!copy_arm_state(
            source.joints, detail::component_ids::joint::kLeftArm, result.left_arm)) {
        return false;
    }
    if (!copy_arm_state(
            source.joints, detail::component_ids::joint::kRightArm, result.right_arm)) {
        return false;
    }
    copy_gripper_state(*left_gripper, result.left_gripper);
    copy_gripper_state(*right_gripper, result.right_gripper);
    copy_base_state(*base, result.base);
    state = std::move(result);
    return true;
}

bool Simulation::read_arm_state(Arm arm, ArmState& state) const {
    romujoco::RobotState source;
    if (!impl_->simulation.read_state(source)) return false;
    ArmState result;
    if (!copy_arm_state(source.joints, arm_ids(arm), result)) return false;
    state = result;
    return true;
}

bool Simulation::read_gripper_state(
    Gripper gripper, GripperState& state) const {
    romujoco::GripperState source;
    source.id = gripper_id(gripper);
    if (!impl_->simulation.read_state(source)) return false;
    copy_gripper_state(source, state);
    return true;
}

bool Simulation::read_spine_state(SpineState& state) const {
    romujoco::JointState source;
    source.id = detail::component_ids::joint::kSpine;
    if (!impl_->simulation.read_state(source)) return false;
    copy_joint_state(source, state);
    return true;
}

bool Simulation::read_base_state(BaseState& state) const {
    romujoco::MobileBaseState source;
    source.id = detail::component_ids::mobile_base::kTmr;
    if (!impl_->simulation.read_state(source)) return false;
    copy_base_state(source, state);
    return true;
}

bool Simulation::read_imu_state(ImuState& state) const {
    romujoco::ImuState source;
    source.id = detail::component_ids::imu::kBase;
    if (!impl_->simulation.read_state(source)) return false;

    ImuState result;
    result.sequence = source.sequence;
    result.timestamp = source.timestamp;
    result.frame_id = source.frame_id;
    result.orientation = {
        source.orientation[0],
        source.orientation[1],
        source.orientation[2],
        source.orientation[3]};
    result.angular_velocity = {
        source.angular_velocity[0],
        source.angular_velocity[1],
        source.angular_velocity[2]};
    result.linear_acceleration = {
        source.linear_acceleration[0],
        source.linear_acceleration[1],
        source.linear_acceleration[2]};
    result.orientation_covariance = source.orientation_covariance;
    result.angular_velocity_covariance = source.angular_velocity_covariance;
    result.linear_acceleration_covariance = source.linear_acceleration_covariance;
    state = std::move(result);
    return true;
}

bool Simulation::read_camera(Camera camera, CameraFrame& frame) const {
    romujoco::CameraState source;
    source.id = camera_id(camera);
    if (!impl_->simulation.read_state(source)) return false;

    CameraFrame result;
    result.sequence = source.sequence;
    result.timestamp = source.timestamp;
    result.frame_id = source.frame_id;
    result.optical_frame_id = source.optical_frame_id;
    result.image = copy_image(source.image);
    result.depth_image = copy_image(source.depth_image);
    result.camera_info = copy_camera_info(source.camera_info);
    frame = std::move(result);
    return true;
}

bool Simulation::read_lidar(Lidar lidar, LaserScan& scan) const {
    romujoco::LaserScanState source;
    source.id = lidar_id(lidar);
    if (!impl_->simulation.read_state(source)) return false;

    LaserScan result;
    result.sequence = source.sequence;
    result.timestamp = source.scan.timestamp;
    result.frame_id = source.scan.frame_id;
    result.angle_min = source.scan.angle_min;
    result.angle_max = source.scan.angle_max;
    result.angle_increment = source.scan.angle_increment;
    result.time_increment = source.scan.time_increment;
    result.scan_time = source.scan.scan_time;
    result.range_min = source.scan.range_min;
    result.range_max = source.scan.range_max;
    result.ranges = source.scan.ranges;
    result.intensities = source.scan.intensities;
    scan = std::move(result);
    return true;
}

bool Simulation::step(std::size_t count) {
    if (impl_->simulation.status() != romujoco::SimulationStatus::Stopped) {
        return false;
    }
    return impl_->simulation.step(count);
}

std::uint64_t Simulation::step_count() const { return impl_->simulation.step_count(); }

double Simulation::time() const { return impl_->simulation.time(); }

SimulationStatus Simulation::status() const {
    return from_romujoco_status(impl_->simulation.status());
}

}  // namespace mfr3duo_mujoco