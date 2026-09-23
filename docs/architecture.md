# mfr3duo_mujoco Architecture

## Scope

mfr3duo_mujoco is the robot-specific integration layer between:

- mfr3duo_description: authoritative URDF/MJCF and meshes.
- robot_mujoco/romujoco: generic MuJoCo runtime and device components.

The package does not copy MJCF assets and does not implement generic MuJoCo components. It owns only MFR3Duo-specific component assembly, runtime defaults, bringup and whole-robot validation.

Architecture:

    mfr3duo_description
            |
            | scene.xml / mfr3duo.xml
            v
    mfr3duo_mujoco
            | robot-specific assembly
            v
    romujoco::Simulation
            |
            v
          MuJoCo

ROS 2 protocol adaptation belongs to the generic robot_mujoco/ros2_mujoco layer. It should be consumed here rather than reimplemented once all required adapters, including Gripper, are available.

## V1 component assembly

The default configuration loads mfr3duo_description/mjcf/scene.xml and registers:

- 15 active joints: spine plus left/right FR3.
- 5 passive TMR joints: two caster pairs plus rocker arm.
- 2 Franka Hand grippers.
- 1 dynamic two-module swerve mobile base.
- 1 IMU.
- 2 nanoScan3 laser scanners.
- 14 MuJoCo camera objects covering four D455 units, two wrist D435 units and the ZED Mini stereo pair.

The four active TMR steering/drive joints are owned exclusively by MobileBase. They are intentionally not duplicated as Joint components.

## Model ownership

mfr3duo_description remains the single source of truth for geometry, mass/inertia, joint topology, actuator definitions, contacts, sensors, camera poses and keyframes. mfr3duo_mujoco contains only runtime-facing parameters that are not expressed by the generic component interface.

Model loading uses the ROS 2 ament index at runtime and resolves:

    share/mfr3duo_description/mjcf/scene.xml

This avoids model copies, symlinks and fragile package-relative assumptions.

## Control baseline

FR3 joints use the frozen Franka position, velocity and effort limits already present in mfr3duo_description. The default runtime controller is position mode with the 7000 stiffness baseline encoded in the frozen Franka parameter data. Command effort remains limited by the corresponding MJCF actuator limits.

The spine uses the MJCF force capacity of +/-600 N rather than the smaller URDF interface limit because the MuJoCo model must support the complete upper structure. Gravity compensation is enabled only for the spine. Enabling it independently for all 14 arm joints would force repeated full-model gravity computations in the current romujoco Joint implementation.

Grippers use the actual one-actuator-plus-equality-coupling topology from mfr3duo.xml.

## Sensor baseline

The two nanoScan3 components use a 275 degree field, 0.17 degree angular resolution, 25 Hz period and 40 m distance range. The 0.05 m minimum range is an integration-layer near-range guard. Raycasts include environment and collision geometry groups while excluding rendering-only visual geometry.

Camera components preserve the individual MuJoCo camera objects instead of treating a RealSense RGB/depth pair as one pinhole camera. This is required because color and depth cameras have different poses and fields of view in the MJCF.

A current upstream romujoco limitation remains for depth-only CameraInfo: width and height are populated from the RGB buffer. The depth image itself is rendered from the correct MJCF depth camera. This package deliberately does not hide that runtime defect with a robot-specific workaround.

## Validation

Two levels are provided:

1. mfr3duo_mujoco.config checks the complete assembly topology without loading MuJoCo.
2. mfr3duo_mujoco.simulation_smoke loads the installed real scene.xml, initializes the whole robot with camera rendering disabled, advances physics and verifies all core state groups.

The smoke test is intentionally not environment-variable gated. If mfr3duo_description and romujoco become incompatible, this package should fail instead of silently skipping coverage.
