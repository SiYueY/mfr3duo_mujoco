#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "kinematics.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

bool limits_valid(const mfr3duo_mujoco::teleop::ArmJointLimits& limits) {
    for (std::size_t index = 0; index < mfr3duo_mujoco::kArmJointCount; ++index) {
        if (!std::isfinite(limits.lower[index]) ||
            !std::isfinite(limits.upper[index]) ||
            !std::isfinite(limits.velocity[index]) ||
            limits.lower[index] >= limits.upper[index] ||
            limits.velocity[index] <= 0.0) {
            return false;
        }
    }
    return true;
}

bool jacobian_valid(const mfr3duo_mujoco::teleop::ArmJacobian& jacobian) {
    bool nonzero = false;
    for (double value : jacobian.values) {
        if (!std::isfinite(value)) return false;
        nonzero = nonzero || std::abs(value) > 1e-9;
    }
    return nonzero;
}

}  // namespace

int main() {
    using namespace mfr3duo_mujoco;
    using namespace mfr3duo_mujoco::teleop;

    Kinematics kinematics;
    if (!check(kinematics.initialize(MFR3DUO_TEST_URDF_PATH), "failed to load URDF")) {
        return EXIT_FAILURE;
    }

    ArmJointLimits left_limits;
    ArmJointLimits right_limits;
    if (!check(kinematics.read_limits(Arm::Left, left_limits), "left limits") ||
        !check(kinematics.read_limits(Arm::Right, right_limits), "right limits") ||
        !check(limits_valid(left_limits), "invalid left limits") ||
        !check(limits_valid(right_limits), "invalid right limits")) {
        return EXIT_FAILURE;
    }

    const std::array<double, kArmJointCount> left_home{
        0.0261, -1.2125, -0.6070, -0.9266, 0.0010, 0.8389, 0.0281};
    const std::array<double, kArmJointCount> right_home{
        -0.2357, -1.1902, -0.2823, -1.2111, 0.3914, 0.4848, -0.2972};

    ArmJacobian jacobian;
    const bool passed =
        check(
            kinematics.compute_jacobian(
                Arm::Left, 0.2, left_home, CartesianFrame::Base, jacobian),
            "left base-frame Jacobian") &&
        check(jacobian_valid(jacobian), "invalid left base-frame Jacobian") &&
        check(
            kinematics.compute_jacobian(
                Arm::Left, 0.2, left_home, CartesianFrame::Tool, jacobian),
            "left tool-frame Jacobian") &&
        check(jacobian_valid(jacobian), "invalid left tool-frame Jacobian") &&
        check(
            kinematics.compute_jacobian(
                Arm::Right, 0.2, right_home, CartesianFrame::Base, jacobian),
            "right base-frame Jacobian") &&
        check(jacobian_valid(jacobian), "invalid right base-frame Jacobian") &&
        check(
            !kinematics.read_limits(static_cast<Arm>(255), left_limits),
            "invalid arm selector unexpectedly accepted");

    return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
