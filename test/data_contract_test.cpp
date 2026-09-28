#include <cstdlib>
#include <iostream>
#include <type_traits>
#include <utility>

#include "mfr3duo_mujoco/data/robot.hpp"

namespace {

template <typename T, typename = void>
struct HasImu : std::false_type {};

template <typename T>
struct HasImu<T, std::void_t<decltype(std::declval<T>().imu)>> : std::true_type {};

static_assert(static_cast<std::uint8_t>(mfr3duo_mujoco::JointControlMode::Hybrid) == 0);
static_assert(static_cast<std::uint8_t>(mfr3duo_mujoco::JointControlMode::Position) == 1);
static_assert(static_cast<std::uint8_t>(mfr3duo_mujoco::JointControlMode::Velocity) == 2);
static_assert(static_cast<std::uint8_t>(mfr3duo_mujoco::JointControlMode::Effort) == 3);
static_assert(std::is_same_v<
              std::underlying_type_t<mfr3duo_mujoco::JointControlMode>, std::uint8_t>);
static_assert(!HasImu<mfr3duo_mujoco::RobotState>::value);
static_assert(!HasImu<mfr3duo_mujoco::RobotCommand>::value);
static_assert(std::is_same_v<mfr3duo_mujoco::SpineCommand, mfr3duo_mujoco::JointCommand>);
static_assert(std::is_same_v<mfr3duo_mujoco::SpineState, mfr3duo_mujoco::JointState>);

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main() {
    using namespace mfr3duo_mujoco;
    const JointCommand command;
    const JointState state;
    const RobotCommand robot_command;
    const RobotState robot_state;
    return check(command.mode == JointControlMode::Position, "joint command default mode") &&
                   check(state.mode == JointControlMode::Position, "joint state default mode") &&
                   check(robot_command.left_arm.joints[0].mode == JointControlMode::Position,
                         "robot command default mode") &&
                   check(robot_state.right_arm.joints[0].mode == JointControlMode::Position,
                         "robot state default mode") &&
                   check(robot_state.base.pose.orientation.w == 1.0,
                         "base orientation default")
               ? EXIT_SUCCESS
               : EXIT_FAILURE;
}
