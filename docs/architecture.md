# mfr3duo_mujoco Architecture

## Scope and dependency direction

mfr3duo_mujoco is the robot-specific simulation layer between the authoritative MFR3Duo model and application code:

    User C++ application
            |
            v
    mfr3duo_mujoco::Simulation
            |
            | MFR3Duo robot semantics
            v
    romujoco::Simulation
            |
            v
          MuJoCo

mfr3duo_description remains the source of truth for MJCF, meshes, dynamics, sensors and keyframes. romujoco remains the generic MuJoCo runtime. mfr3duo_mujoco owns the MFR3Duo assembly and maps robot-level commands and states to generic romujoco components.

The core is a standalone C++17/CMake library. It does not depend on ROS 2, ament, DDS or ros2_control. Those systems may be added later only as adapters above the public C++ API.

## Public API boundary

The public API is organized by meaning rather than by transport or middleware:

    simulation.hpp
        lifecycle, stepping and robot access

    config.hpp
        runtime options and scene discovery

    command.hpp
        ArmCommand
        SpineCommand
        GripperCommand
        BaseCommand

    state.hpp
        RobotState
        ArmState
        SpineState
        GripperState
        BaseState
        ImuState

    camera.hpp
        Camera
        CameraFrame

    lidar.hpp
        Lidar
        LaserScan

Camera and LiDAR payloads are deliberately excluded from RobotState. A high-rate control loop can therefore read RobotState without copying image or scan buffers.

romujoco component IDs, actuator names and SimulationConfig are private implementation details under src/.

## Simulation execution

The public API deliberately exposes no execution-mode enum. The caller selects the
execution style by which operation it invokes:

- `step(count)` explicitly advances a stopped simulation by a fixed number of
  physics steps. This is intended for controllers, agents, tests and deterministic
  experiments that own simulation-time progression.
- `start()` starts continuous execution using the internal scheduler. The caller
  can continue reading state and writing commands while the simulation advances.

The two styles are mutually exclusive. `step()` is accepted only while the
simulation status is `Stopped`; it is rejected while `Running`, `Paused`,
`Stopping`, `Error` or `Uninitialized`. Pausing continuous execution does not
transfer ownership of time advancement to the caller.

## Control mapping

The robot-level API exposes explicit MFR3Duo semantics:

- Arm::Left and Arm::Right each map to seven FR3 active joints.
- the spine maps to franka_spine_vertical_joint.
- Gripper::Left and Gripper::Right map to the two Franka Hands.
- BaseCommand maps to the dynamic TMR swerve component.

Arm commands are submitted to romujoco as one seven-joint batch. This preserves a coherent arm command update instead of requiring application code to issue seven unrelated component-ID writes.

The API supports Position, Velocity, Effort and Hybrid joint modes. Position and velocity controller gains remain robot integration parameters. Hybrid stiffness and damping are supplied with each JointCommand because romujoco defines them as command data.

## State mapping

RobotState is one coherent low-bandwidth snapshot containing:

- simulation sequence, timestamp, time and step;
- spine;
- both seven-axis arms;
- both grippers;
- mobile-base ground-truth pose and twist.

The base quaternion is normalized at the public boundary to x/y/z/w field semantics even though MuJoCo free-joint storage is w/x/y/z.

IMU is read separately because it can be disabled independently. Camera and LiDAR have dedicated APIs and also return false when the corresponding component is disabled.

## Component assembly

The canonical configuration registers:

- 15 active joints: spine plus both seven-axis FR3 arms;
- 5 passive TMR joints;
- 2 Franka Hand grippers;
- 1 dynamic two-module swerve base;
- 1 IMU;
- 2 nanoScan3 scanners;
- 14 MuJoCo camera objects.

The active TMR steering and drive joints belong exclusively to MobileBase and are not duplicated as Joint components.

Internal component IDs are stable only inside this integration layer. They are not an external control contract.

## Model discovery

The default Simulation::initialize() resolves mfr3duo_description/mjcf/scene.xml without ament. Resolution checks MFR3DUO_DESCRIPTION_PATH, CMAKE_PREFIX_PATH and an optional description directory discovered at build time.

Applications that manage model paths themselves can call the explicit-path initialize overload, which avoids discovery completely.

## Validation

The configuration test verifies the complete 40-component assembly without loading MuJoCo.

The simulation smoke test loads the real scene with camera rendering disabled, advances at least 50 physics steps so the 25 Hz LiDARs have produced samples, and verifies the public robot state, IMU and both LiDAR interfaces.

Camera RGB/depth objects remain separate because the MJCF models them as distinct cameras. The existing upstream depth-only CameraInfo metadata limitation remains a romujoco issue and is not hidden with a robot-specific workaround.