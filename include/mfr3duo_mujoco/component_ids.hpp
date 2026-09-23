#pragma once

#include "romujoco/component/camera.hpp"
#include "romujoco/component/component_id.hpp"
#include "romujoco/component/gripper.hpp"
#include "romujoco/component/imu.hpp"
#include "romujoco/component/joint.hpp"
#include "romujoco/component/lidar.hpp"

namespace mfr3duo_mujoco::component_ids {

namespace joint {
inline constexpr romujoco::JointId kSpine = 0;

inline constexpr romujoco::JointId kLeftJoint1 = 1;
inline constexpr romujoco::JointId kLeftJoint2 = 2;
inline constexpr romujoco::JointId kLeftJoint3 = 3;
inline constexpr romujoco::JointId kLeftJoint4 = 4;
inline constexpr romujoco::JointId kLeftJoint5 = 5;
inline constexpr romujoco::JointId kLeftJoint6 = 6;
inline constexpr romujoco::JointId kLeftJoint7 = 7;

inline constexpr romujoco::JointId kRightJoint1 = 8;
inline constexpr romujoco::JointId kRightJoint2 = 9;
inline constexpr romujoco::JointId kRightJoint3 = 10;
inline constexpr romujoco::JointId kRightJoint4 = 11;
inline constexpr romujoco::JointId kRightJoint5 = 12;
inline constexpr romujoco::JointId kRightJoint6 = 13;
inline constexpr romujoco::JointId kRightJoint7 = 14;

inline constexpr romujoco::JointId kCasterFrontLeftSteering = 15;
inline constexpr romujoco::JointId kCasterFrontLeftWheel = 16;
inline constexpr romujoco::JointId kRockerArm = 17;
inline constexpr romujoco::JointId kCasterRearRightSteering = 18;
inline constexpr romujoco::JointId kCasterRearRightWheel = 19;
}  // namespace joint

namespace gripper {
inline constexpr romujoco::GripperId kLeft = 0;
inline constexpr romujoco::GripperId kRight = 1;
}  // namespace gripper

namespace mobile_base {
inline constexpr romujoco::ComponentId kTmr = 0;
}  // namespace mobile_base

namespace imu {
inline constexpr romujoco::ImuId kBase = 0;
}  // namespace imu

namespace lidar {
inline constexpr romujoco::LidarId kFront = 0;
inline constexpr romujoco::LidarId kRear = 1;
}  // namespace lidar

namespace camera {
inline constexpr romujoco::CameraId kFrontColor = 0;
inline constexpr romujoco::CameraId kFrontDepth = 1;
inline constexpr romujoco::CameraId kRearColor = 2;
inline constexpr romujoco::CameraId kRearDepth = 3;
inline constexpr romujoco::CameraId kRightColor = 4;
inline constexpr romujoco::CameraId kRightDepth = 5;
inline constexpr romujoco::CameraId kLeftColor = 6;
inline constexpr romujoco::CameraId kLeftDepth = 7;
inline constexpr romujoco::CameraId kLeftWristColor = 8;
inline constexpr romujoco::CameraId kLeftWristDepth = 9;
inline constexpr romujoco::CameraId kRightWristColor = 10;
inline constexpr romujoco::CameraId kRightWristDepth = 11;
inline constexpr romujoco::CameraId kHeadZedLeft = 12;
inline constexpr romujoco::CameraId kHeadZedRight = 13;
}  // namespace camera

}  // namespace mfr3duo_mujoco::component_ids
