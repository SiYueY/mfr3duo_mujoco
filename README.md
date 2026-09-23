# mfr3duo_mujoco

mfr3duo_mujoco is the Mobile FR3 Duo integration package for MuJoCo.

It combines the authoritative model from mfr3duo_description with the generic simulation runtime from robot_mujoco/romujoco. The package does not copy the MJCF and does not reimplement generic simulation components.

## Responsibilities

This package owns only robot-specific integration:

- locate and load mfr3duo_description/mjcf/scene.xml;
- assemble all MFR3Duo romujoco components;
- provide a runnable whole-robot simulator;
- provide whole-robot compatibility tests;
- later provide MFR3Duo ROS 2 bringup by reusing the generic ros2_mujoco adapter.

The default assembly includes the TMR dynamic swerve base, spine, both 7-DoF FR3 arms, both Franka Hands, IMU, two nanoScan3 scanners and all 14 MJCF camera objects.

See docs/architecture.md for ownership and component mapping.

## Dependencies

- Ubuntu Linux
- ROS 2 and ament_cmake
- mfr3duo_description
- an installed romujoco CMake package

romujoco is a standalone CMake package rather than an ament package. Install it first and expose its prefix through CMAKE_PREFIX_PATH.

Example for robot_mujoco/romujoco:

    ./scripts/mujoco.sh build
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$HOME/.local/romujoco"
    cmake --build build -j
    cmake --install build

    export CMAKE_PREFIX_PATH="$HOME/.local/romujoco:$CMAKE_PREFIX_PATH"

Then build mfr3duo_description and this package in a ROS 2 workspace:

    colcon build --packages-up-to mfr3duo_mujoco
    source install/setup.bash

## Run

Start the complete simulator with the interactive viewer:

    ros2 run mfr3duo_mujoco mfr3duo_sim

Headless operation:

    ros2 run mfr3duo_mujoco mfr3duo_sim --headless

For a lightweight core check without GPU camera rendering:

    ros2 run mfr3duo_mujoco mfr3duo_sim --headless --no-cameras --steps 1000

Available options:

    --headless
    --no-cameras
    --no-lidars
    --no-imu
    --camera-width N
    --camera-height N
    --keyframe NAME
    --steps N

The default initial keyframe is home.

## Tests

Run:

    colcon test --packages-select mfr3duo_mujoco
    colcon test-result --verbose

The package includes a smoke test against the real installed MFR3Duo MJCF. It does not silently skip when the model is missing or incompatible.

## Current boundary

The MuJoCo core assembly is the V1 target. ROS 2 message and ros2_control adaptation belongs to robot_mujoco/ros2_mujoco and should not be duplicated here. Full ROS 2 bringup should be added after the generic adapter exposes the new romujoco Gripper component.

Camera RGB and depth objects are configured separately because mfr3duo_description models them as distinct MuJoCo cameras with distinct poses and fields of view. A known upstream romujoco issue remains in depth-only CameraInfo metadata; the depth image itself still comes from the correct MJCF camera.
