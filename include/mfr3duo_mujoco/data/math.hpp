#pragma once

namespace mfr3duo_mujoco {

/** @brief Three-dimensional vector with Cartesian components. */
struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/** @brief Quaternion in x/y/z/w field order. */
struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

/** @brief Position and orientation. */
struct Pose {
    Vector3 position;
    Quaternion orientation;
};

/** @brief Linear and angular velocity. */
struct Twist {
    Vector3 linear;
    Vector3 angular;
};

}  // namespace mfr3duo_mujoco
