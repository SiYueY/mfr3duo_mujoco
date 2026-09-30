#include "mfr3duo_mujoco/simulation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <utility>

#include "component_ids.hpp"
#include "configuration.hpp"
#include "romujoco/simulation.hpp"

namespace mfr3duo_mujoco {
namespace {

romujoco::JointMode to_romujoco_mode(JointControlMode mode) {
    switch (mode) {
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
        case romujoco::JointMode::Velocity:
            return JointControlMode::Velocity;
        case romujoco::JointMode::Effort:
            return JointControlMode::Effort;
        case romujoco::JointMode::Position:
        case romujoco::JointMode::Hybrid:
        case romujoco::JointMode::None:
            return JointControlMode::Position;
    }
    return JointControlMode::Position;
}

bool valid_arm(Arm arm) noexcept {
    return arm == Arm::Left || arm == Arm::Right;
}

bool valid_gripper(Gripper gripper) noexcept {
    return gripper == Gripper::Left || gripper == Gripper::Right;
}

bool valid_camera(Camera camera) noexcept {
    return static_cast<std::uint8_t>(camera) <=
           static_cast<std::uint8_t>(Camera::HeadZedRight);
}

bool valid_lidar(Lidar lidar) noexcept {
    return lidar == Lidar::Front || lidar == Lidar::Rear;
}

romujoco::JointCommand make_joint_command(
    romujoco::JointId id, const JointCommand& command) {
    romujoco::JointCommand result;
    result.id = id;
    result.mode = static_cast<std::uint8_t>(to_romujoco_mode(command.mode));
    result.position = command.position;
    result.velocity = command.velocity;
    result.effort = command.effort;
    return result;
}

romujoco::GripperCommand make_gripper_command(
    romujoco::GripperId id, const GripperCommand& command) {
    romujoco::GripperCommand result;
    result.id = id;
    result.width = command.width;
    result.velocity = command.velocity;
    result.effort = command.effort;
    return result;
}

void append_tmr_commands(
    const TmrCommand& command, romujoco::JointCommands& result) {
    namespace ids = detail::component_ids::joint::tmr;
    JointCommand steering;
    steering.position = command.front_steering_position;
    result.emplace_back(make_joint_command(ids::kFrontSteering, steering));
    steering.position = command.rear_steering_position;
    result.emplace_back(make_joint_command(ids::kRearSteering, steering));

    JointCommand drive;
    drive.mode = JointControlMode::Velocity;
    drive.velocity = command.front_drive_velocity;
    result.emplace_back(make_joint_command(ids::kFrontDrive, drive));
    drive.velocity = command.rear_drive_velocity;
    result.emplace_back(make_joint_command(ids::kRearDrive, drive));
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

bool copy_tmr_state(const romujoco::JointStates& states, TmrState& target) {
    namespace ids = detail::component_ids::joint::tmr;
    const auto* front_steering = find_state(states, ids::kFrontSteering);
    const auto* front_drive = find_state(states, ids::kFrontDrive);
    const auto* rear_steering = find_state(states, ids::kRearSteering);
    const auto* rear_drive = find_state(states, ids::kRearDrive);
    if (front_steering == nullptr || front_drive == nullptr ||
        rear_steering == nullptr || rear_drive == nullptr) return false;
    target.timestamp = front_steering->timestamp;
    copy_joint_state(*front_steering, target.front_steering);
    copy_joint_state(*front_drive, target.front_drive);
    copy_joint_state(*rear_steering, target.rear_steering);
    copy_joint_state(*rear_drive, target.rear_drive);
    return true;
}

bool copy_tmr_passive_state(
    const romujoco::JointStates& states, TmrPassiveState& target) {
    namespace ids = detail::component_ids::joint::tmr;
    const auto* rocker = find_state(states, ids::kRockerArm);
    const auto* front_steering = find_state(states, ids::kFrontCasterSteering);
    const auto* front_wheel = find_state(states, ids::kFrontCasterWheel);
    const auto* rear_steering = find_state(states, ids::kRearCasterSteering);
    const auto* rear_wheel = find_state(states, ids::kRearCasterWheel);
    if (rocker == nullptr || front_steering == nullptr || front_wheel == nullptr ||
        rear_steering == nullptr || rear_wheel == nullptr) return false;
    target.timestamp = rocker->timestamp;
    target.rocker_arm = {rocker->position, rocker->velocity};
    target.front_caster_steering = {front_steering->position, front_steering->velocity};
    target.front_caster_wheel = {front_wheel->position, front_wheel->velocity};
    target.rear_caster_steering = {rear_steering->position, rear_steering->velocity};
    target.rear_caster_wheel = {rear_wheel->position, rear_wheel->velocity};
    return true;
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
    // Reuse conversion storage without sharing a mutable command across callers.
    std::mutex robot_command_mutex;
    romujoco::RobotCommand robot_command;
};

Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}

Simulation::~Simulation() = default;

bool Simulation::initialize(const SimulationOptions& options) {
    try {
        return impl_->simulation.initialize(detail::make_simulation_config(options));
    } catch (const std::exception&) {
        return false;
    }
}

bool Simulation::initialize(
    const std::string& model_path, const SimulationOptions& options) {
    try {
        return impl_->simulation.initialize(
            detail::make_simulation_config(model_path, options));
    } catch (const std::exception&) {
        return false;
    }
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

bool Simulation::write_command(Arm arm, const ArmCommand& command) {
    if (!valid_arm(arm)) return false;
    const auto& ids = arm_ids(arm);
    romujoco::JointCommands commands;
    commands.reserve(kArmJointCount);
    for (std::size_t index = 0; index < ids.size(); ++index) {
        commands.emplace_back(make_joint_command(ids[index], command.joints[index]));
    }
    return impl_->simulation.write_commands(commands);
}

bool Simulation::write_command(
    Gripper gripper, const GripperCommand& command) {
    if (!valid_gripper(gripper)) return false;
    return impl_->simulation.write_command(
        make_gripper_command(gripper_id(gripper), command));
}

bool Simulation::write_command(const SpineCommand& command) {
    return impl_->simulation.write_command(
        make_joint_command(detail::component_ids::joint::kSpine, command));
}

bool Simulation::write_command(const TmrCommand& command) {
    romujoco::JointCommands commands;
    commands.reserve(4);
    append_tmr_commands(command, commands);
    return impl_->simulation.write_commands(commands);
}

bool Simulation::write_command(const RobotCommand& command) {
    std::lock_guard<std::mutex> lock(impl_->robot_command_mutex);
    auto& result = impl_->robot_command;
    result.joints.clear();
    result.joints.reserve(1 + 2 * kArmJointCount + 4);
    result.joints.emplace_back(
        make_joint_command(detail::component_ids::joint::kSpine, command.spine));
    for (std::size_t index = 0; index < kArmJointCount; ++index) {
        result.joints.emplace_back(make_joint_command(
            detail::component_ids::joint::kLeftArm[index],
            command.left_arm.joints[index]));
        result.joints.emplace_back(make_joint_command(
            detail::component_ids::joint::kRightArm[index],
            command.right_arm.joints[index]));
    }
    append_tmr_commands(command.tmr, result.joints);

    result.grippers.clear();
    result.grippers.reserve(2);
    result.grippers.emplace_back(make_gripper_command(
        detail::component_ids::gripper::kLeft, command.left_gripper));
    result.grippers.emplace_back(make_gripper_command(
        detail::component_ids::gripper::kRight, command.right_gripper));
    return impl_->simulation.write_command(result);
}

bool Simulation::read_state(RobotState& state) const {
    std::shared_ptr<const romujoco::RobotState> source;
    if (!impl_->simulation.read_state(source) || source == nullptr) return false;

    const auto* spine =
        find_state(source->joints, detail::component_ids::joint::kSpine);
    const auto* left_gripper =
        find_state(source->grippers, detail::component_ids::gripper::kLeft);
    const auto* right_gripper =
        find_state(source->grippers, detail::component_ids::gripper::kRight);
    if (spine == nullptr || left_gripper == nullptr || right_gripper == nullptr) {
        return false;
    }

    RobotState result;
    result.sequence = source->sequence;
    result.timestamp = source->timestamp;
    result.simulation_time = source->simulation_time;
    result.step = source->step;
    copy_joint_state(*spine, result.spine);
    if (!copy_arm_state(
            source->joints, detail::component_ids::joint::kLeftArm, result.left_arm)) {
        return false;
    }
    if (!copy_arm_state(
            source->joints, detail::component_ids::joint::kRightArm, result.right_arm)) {
        return false;
    }
    copy_gripper_state(*left_gripper, result.left_gripper);
    copy_gripper_state(*right_gripper, result.right_gripper);
    if (!copy_tmr_state(source->joints, result.tmr)) return false;
    state = std::move(result);
    return true;
}

bool Simulation::read_state(Arm arm, ArmState& state) const {
    if (!valid_arm(arm)) return false;
    std::shared_ptr<const romujoco::RobotState> source;
    if (!impl_->simulation.read_state(source) || source == nullptr) return false;
    ArmState result;
    if (!copy_arm_state(source->joints, arm_ids(arm), result)) return false;
    state = result;
    return true;
}

bool Simulation::read_state(
    Gripper gripper, GripperState& state) const {
    if (!valid_gripper(gripper)) return false;
    romujoco::GripperState source;
    source.id = gripper_id(gripper);
    if (!impl_->simulation.read_state(source)) return false;
    copy_gripper_state(source, state);
    return true;
}

bool Simulation::read_state(SpineState& state) const {
    romujoco::JointState source;
    source.id = detail::component_ids::joint::kSpine;
    if (!impl_->simulation.read_state(source)) return false;
    copy_joint_state(source, state);
    return true;
}

bool Simulation::read_state(TmrState& state) const {
    std::shared_ptr<const romujoco::RobotState> source;
    if (!impl_->simulation.read_state(source) || source == nullptr) return false;
    TmrState result;
    if (!copy_tmr_state(source->joints, result)) return false;
    state = result;
    return true;
}

bool Simulation::read_state(TmrPassiveState& state) const {
    std::shared_ptr<const romujoco::RobotState> source;
    if (!impl_->simulation.read_state(source) || source == nullptr) return false;
    TmrPassiveState result;
    if (!copy_tmr_passive_state(source->joints, result)) return false;
    state = result;
    return true;
}

bool Simulation::read_state(ImuState& state) const {
    std::shared_ptr<const romujoco::RobotState> snapshot;
    if (!impl_->simulation.read_state(snapshot) || snapshot == nullptr) return false;
    const auto* source = find_state(snapshot->imus, detail::component_ids::imu::kBase);
    if (source == nullptr) return false;

    state.sequence = source->sequence;
    state.timestamp = source->timestamp;
    state.frame_id = source->frame_id;
    state.orientation = {
        source->orientation[0],
        source->orientation[1],
        source->orientation[2],
        source->orientation[3]};
    state.angular_velocity = {
        source->angular_velocity[0],
        source->angular_velocity[1],
        source->angular_velocity[2]};
    state.linear_acceleration = {
        source->linear_acceleration[0],
        source->linear_acceleration[1],
        source->linear_acceleration[2]};
    state.orientation_covariance = source->orientation_covariance;
    state.angular_velocity_covariance = source->angular_velocity_covariance;
    state.linear_acceleration_covariance = source->linear_acceleration_covariance;
    return true;
}

bool Simulation::read_state(Camera camera, CameraFrame& frame) const {
    if (!valid_camera(camera)) return false;
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

bool Simulation::read_state(Lidar lidar, LaserScan& scan) const {
    if (!valid_lidar(lidar)) return false;
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
