#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "mfr3duo_mujoco/camera.hpp"
#include "mfr3duo_mujoco/command.hpp"
#include "mfr3duo_mujoco/config.hpp"
#include "mfr3duo_mujoco/lidar.hpp"
#include "mfr3duo_mujoco/state.hpp"

namespace mfr3duo_mujoco {

enum class Arm : std::uint8_t {
    Left,
    Right,
};

enum class Gripper : std::uint8_t {
    Left,
    Right,
};

enum class SimulationStatus : std::uint8_t {
    Uninitialized,
    Stopped,
    Running,
    Paused,
    Stopping,
    Error,
};

/**
 * @brief Mobile FR3 Duo robot-level MuJoCo simulation API.
 *
 * Simulation owns exactly one romujoco simulation internally. Callers control
 * the robot through MFR3Duo semantics and never need component IDs or MuJoCo
 * actuator names.
 */
class Simulation {
public:
    Simulation();
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) = delete;
    Simulation& operator=(Simulation&&) = delete;

    /**
     * @brief Initialize the canonical installed MFR3Duo scene.
     */
    bool initialize(const SimulationOptions& options = {});

    /**
     * @brief Initialize from an explicit MFR3Duo-compatible MJCF scene.
     */
    bool initialize(const std::string& model_path, const SimulationOptions& options = {});

    bool shutdown();

    /**
     * @brief Start continuous simulation execution.
     *
     * The internal scheduler advances simulation time until stop() is called.
     * start() is valid only while the simulation is stopped.
     */
    bool start();
    bool stop();
    bool pause();
    bool resume();
    bool reset();
    bool reset(const std::string& keyframe_name);

    bool write_command(Arm arm, const ArmCommand& command);
    bool write_command(Gripper gripper, const GripperCommand& command);
    bool write_command(const SpineCommand& command);
    bool write_command(const BaseCommand& command);

    bool read_state(RobotState& state) const;
    bool read_state(Arm arm, ArmState& state) const;
    bool read_state(Gripper gripper, GripperState& state) const;
    bool read_state(SpineState& state) const;
    bool read_state(BaseState& state) const;
    bool read_state(ImuState& state) const;
    bool read_state(Camera camera, CameraFrame& frame) const;
    bool read_state(Lidar lidar, LaserScan& scan) const;

    /**
     * @brief Advance the simulation explicitly by a fixed number of physics steps.
     *
     * step() is valid only while the simulation is stopped. It cannot be mixed
     * with continuous execution started by start(), including while paused.
     */
    bool step(std::size_t count = 1);
    std::uint64_t step_count() const;
    double time() const;
    SimulationStatus status() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mfr3duo_mujoco