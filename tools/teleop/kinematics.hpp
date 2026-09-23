#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include "mfr3duo_mujoco/simulation.hpp"

namespace mfr3duo_mujoco::teleop {

enum class CartesianFrame {
    Base,
    Tool,
};

struct ArmJointLimits {
    std::array<double, kArmJointCount> lower{};
    std::array<double, kArmJointCount> upper{};
    std::array<double, kArmJointCount> velocity{};
};

struct ArmJacobian {
    std::array<double, 6 * kArmJointCount> values{};

    double operator()(std::size_t row, std::size_t column) const {
        return values[row * kArmJointCount + column];
    }
};

class Kinematics {
public:
    Kinematics();
    ~Kinematics();

    Kinematics(const Kinematics&) = delete;
    Kinematics& operator=(const Kinematics&) = delete;
    Kinematics(Kinematics&&) = delete;
    Kinematics& operator=(Kinematics&&) = delete;

    bool initialize(const std::string& urdf_path);

    bool read_limits(Arm arm, ArmJointLimits& limits) const;

    bool compute_jacobian(
        Arm arm,
        double spine_position,
        const std::array<double, kArmJointCount>& joint_position,
        CartesianFrame frame,
        ArmJacobian& jacobian);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mfr3duo_mujoco::teleop
