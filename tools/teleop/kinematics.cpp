#include "pinocchio/fwd.hpp"

#include "kinematics.hpp"

#include <array>
#include <exception>
#include <string>

#include <Eigen/Core>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace mfr3duo_mujoco::teleop {
namespace {

constexpr const char* kSpineJoint = "franka_spine_vertical_joint";

constexpr std::array<const char*, kArmJointCount> kLeftArmJoints = {
    "left_fr3v2_1_joint1",
    "left_fr3v2_1_joint2",
    "left_fr3v2_1_joint3",
    "left_fr3v2_1_joint4",
    "left_fr3v2_1_joint5",
    "left_fr3v2_1_joint6",
    "left_fr3v2_1_joint7",
};

constexpr std::array<const char*, kArmJointCount> kRightArmJoints = {
    "right_fr3v2_1_joint1",
    "right_fr3v2_1_joint2",
    "right_fr3v2_1_joint3",
    "right_fr3v2_1_joint4",
    "right_fr3v2_1_joint5",
    "right_fr3v2_1_joint6",
    "right_fr3v2_1_joint7",
};

constexpr const char* kLeftToolFrame = "left_fr3v2_1_hand_tcp";
constexpr const char* kRightToolFrame = "right_fr3v2_1_hand_tcp";

const std::array<const char*, kArmJointCount>& joint_names(Arm arm) {
    return arm == Arm::Left ? kLeftArmJoints : kRightArmJoints;
}

const char* tool_frame_name(Arm arm) {
    return arm == Arm::Left ? kLeftToolFrame : kRightToolFrame;
}

}  // namespace

class Kinematics::Impl {
public:
    struct ArmModel {
        std::array<int, kArmJointCount> q_indices{};
        std::array<int, kArmJointCount> v_indices{};
        pinocchio::FrameIndex tool_frame{0};
        ArmJointLimits limits;
    };

    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    Eigen::VectorXd neutral;
    int spine_q_index{-1};
    ArmModel left;
    ArmModel right;
    bool initialized{false};

    const ArmModel& arm(Arm value) const {
        return value == Arm::Left ? left : right;
    }

    bool configure_arm(Arm value, ArmModel& target) {
        const auto& names = joint_names(value);
        for (std::size_t index = 0; index < names.size(); ++index) {
            const pinocchio::JointIndex joint_id = model.getJointId(names[index]);
            if (joint_id == 0 || joint_id >= model.njoints) return false;
            if (model.nqs[joint_id] != 1 || model.nvs[joint_id] != 1) return false;

            const int q_index = model.idx_qs[joint_id];
            const int v_index = model.idx_vs[joint_id];
            target.q_indices[index] = q_index;
            target.v_indices[index] = v_index;
            target.limits.lower[index] = model.lowerPositionLimit[q_index];
            target.limits.upper[index] = model.upperPositionLimit[q_index];
            target.limits.velocity[index] = model.upperVelocityLimit[v_index];
        }

        const pinocchio::FrameIndex frame = model.getFrameId(tool_frame_name(value));
        if (frame >= model.nframes) return false;
        target.tool_frame = frame;
        return true;
    }
};

Kinematics::Kinematics() : impl_(std::make_unique<Impl>()) {}

Kinematics::~Kinematics() = default;

bool Kinematics::initialize(const std::string& urdf_path) {
    impl_->initialized = false;
    impl_->data.reset();

    try {
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);

        const pinocchio::JointIndex spine = model.getJointId(kSpineJoint);
        if (spine == 0 || spine >= model.njoints || model.nqs[spine] != 1) {
            return false;
        }

        impl_->model = std::move(model);
        impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
        impl_->neutral = pinocchio::neutral(impl_->model);
        impl_->spine_q_index = impl_->model.idx_qs[spine];

        if (!impl_->configure_arm(Arm::Left, impl_->left) ||
            !impl_->configure_arm(Arm::Right, impl_->right)) {
            impl_->data.reset();
            return false;
        }

        impl_->initialized = true;
        return true;
    } catch (const std::exception&) {
        impl_->data.reset();
        impl_->initialized = false;
        return false;
    }
}

bool Kinematics::read_limits(Arm arm, ArmJointLimits& limits) const {
    if (!impl_->initialized) return false;
    limits = impl_->arm(arm).limits;
    return true;
}

bool Kinematics::compute_jacobian(
    Arm arm,
    double spine_position,
    const std::array<double, kArmJointCount>& joint_position,
    CartesianFrame frame,
    ArmJacobian& jacobian) {
    if (!impl_->initialized || impl_->data == nullptr) return false;

    Eigen::VectorXd q = impl_->neutral;
    q[impl_->spine_q_index] = spine_position;

    const Impl::ArmModel& arm_model = impl_->arm(arm);
    for (std::size_t index = 0; index < joint_position.size(); ++index) {
        q[arm_model.q_indices[index]] = joint_position[index];
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, impl_->model.nv);
    full_jacobian.setZero();

    const pinocchio::ReferenceFrame reference =
        frame == CartesianFrame::Base
            ? pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED
            : pinocchio::ReferenceFrame::LOCAL;

    try {
        pinocchio::computeFrameJacobian(
            impl_->model,
            *impl_->data,
            q,
            arm_model.tool_frame,
            reference,
            full_jacobian);
    } catch (const std::exception&) {
        return false;
    }

    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < kArmJointCount; ++column) {
            jacobian.values[row * kArmJointCount + column] =
                full_jacobian(
                    static_cast<Eigen::Index>(row),
                    arm_model.v_indices[column]);
        }
    }
    return true;
}

}  // namespace mfr3duo_mujoco::teleop