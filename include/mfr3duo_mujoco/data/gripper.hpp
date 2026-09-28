#pragma once

namespace mfr3duo_mujoco {

/**
 * @brief Command for one Franka Hand.
 */
struct GripperCommand {
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};

/**
 * @brief State of one Franka Hand.
 */
struct GripperState {
    double timestamp{0.0};
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool stalled{false};
};

}  // namespace mfr3duo_mujoco
