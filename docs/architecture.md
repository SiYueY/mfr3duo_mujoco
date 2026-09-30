# mfr3duo_mujoco Architecture

## Scope

`mfr3duo_mujoco` is the standalone C++17/CMake simulation library for Mobile FR3 Duo. It combines the authoritative model from `mfr3duo_description` with the generic `romujoco` runtime. The core has no ROS 2 or Pinocchio dependency. Component IDs, MJCF actuator names and `romujoco::SimulationConfig` remain private implementation details.

## Public API

- `simulation.hpp` provides lifecycle, fixed stepping, continuous execution, and typed command/state access.
- `data/joint.hpp` defines the Position, Velocity and Effort modes. `JointState::mode` reports the current mode.
- `data/arm.hpp`, `data/spine.hpp`, `data/gripper.hpp` and `data/tmr.hpp` define motion device data.
- `data/robot.hpp` defines a complete `RobotCommand` and coherent `RobotState` for the spine, both FR3 arms, both grippers and the four active TMR joints.
- `TmrPassiveState` exposes five unactuated TMR joints through a separate read API.
- `data/imu.hpp`, `data/camera.hpp` and `data/lidar.hpp` remain independent sensor channels.

A complete `RobotCommand` converts into one `romujoco::RobotCommand` and is submitted once. Device-level writes are partial updates. All `RobotState` motion members are extracted from one bottom-layer snapshot. Sensor samples and passive TMR state are excluded from that snapshot. To compare device-level reads with a robot snapshot, stop the simulation and use fixed-step execution.

## TMR assembly

The four TMR joints are registered as active `romujoco::Joint` components. Front/rear steering accept Position commands; front/rear drive accept Velocity commands. The MFR3Duo MJCF uses motor actuators because `JointComponent` computes actuator effort for both target modes. The five unactuated caster and rocker joints are passive components. The generic `romujoco::MobileBase` capability remains available to other robots; this robot no longer registers one.

The public API exposes no planar base twist command or ground-truth base pose. Swerve inverse/forward kinematics and odometry belong above this library. The standalone keyboard teleop converts its operator twist intent to steering and drive targets; the core library does not perform this conversion.

## Execution

`step(count)` advances a stopped simulation by a fixed number of physics steps. `start()` uses the internal scheduler for continuous execution. These operations are mutually exclusive. `step()` is rejected while continuous execution is running or paused. Optional IMU, Camera and LiDAR flags affect only their independent read channels, not the motion snapshot.

## Tools

`mfr3duo_sim` runs the canonical scene. `mfr3duo_teleop` owns a `Simulation` and uses project-private Pinocchio for arm limits, Jacobians and Cartesian jogging. Pinocchio is not linked by the core library.

The teleop program resolves arm joint limits and Jacobians from the authoritative URDF. Joint jogging integrates a bounded velocity into position targets; Cartesian jogging uses damped least squares and applies both velocity and position limits before writing arm commands. The keyboard's planar base intent is converted to TMR steering and wheel targets inside teleop.

## Model discovery and validation

Default `Simulation::initialize()` resolves the installed `mfr3duo_description/mjcf/scene.xml` through `MFR3DUO_DESCRIPTION_PATH`, `CMAKE_PREFIX_PATH` or the path found at build time. Callers may pass an explicit model path. No ament dependency is needed by the core library.

Configuration tests verify the active and passive joint assembly, allowed modes and actuator names. Fixed-step integration tests compare device states with one whole-robot snapshot and check TMR steering, drive, stop and reverse feedback. The smoke test loads the scene and checks independent IMU and LiDAR reads. Camera RGB and depth streams remain distinct because the MJCF represents them as distinct cameras.
