# mfr3duo_mujoco

mfr3duo_mujoco is the Mobile FR3 Duo MuJoCo simulation library.

It combines the authoritative model from mfr3duo_description with the generic runtime from robot_mujoco/romujoco and exposes a robot-level C++ API. Applications control the simulated robot directly through mfr3duo_mujoco; they do not need ROS 2, MuJoCo actuator names or romujoco component IDs.

## Public API

The public surface is intentionally small:

- simulation.hpp: lifecycle, stepping, command and state access.
- config.hpp: runtime options and canonical scene resolution.
- command.hpp: arm, spine, gripper and mobile-base commands.
- state.hpp: robot, arm, spine, gripper, base and IMU states.
- camera.hpp: camera selection and frame data.
- lidar.hpp: LiDAR selection and scan data.

Component IDs and romujoco assembly types are implementation details.

A direct control program can use the library like this:

    #include <mfr3duo_mujoco/simulation.hpp>

    mfr3duo_mujoco::Simulation simulation;
    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;

    if (!simulation.initialize(options)) {
        return 1;
    }

    mfr3duo_mujoco::BaseCommand base;
    base.linear_x = 0.2;
    simulation.write_base_command(base);

    simulation.step(1000);

    mfr3duo_mujoco::RobotState state;
    simulation.read_state(state);
    simulation.shutdown();

For a complete compilable example, see examples/control.cpp.

Simulation time can be advanced in two direct ways without configuring an execution mode:

    // Caller advances simulation time explicitly.
    simulation.step();

or:

    // Internal scheduler advances simulation time continuously.
    simulation.start();
    // read/write commands and states...
    simulation.stop();

These forms are mutually exclusive. `step()` is accepted only while the
simulation is stopped; it is rejected while continuous execution is running or paused.

## Dependencies

- Linux
- C++17
- an installed romujoco CMake package
- mfr3duo_description model files at runtime

The core library does not depend on ROS 2, ament, DDS or ros2_control.

The canonical scene resolver searches:

1. MFR3DUO_DESCRIPTION_PATH;
2. CMAKE_PREFIX_PATH entries for share/mfr3duo_description/mjcf/scene.xml;
3. MFR3DUO_DESCRIPTION_SHARE_DIR discovered while building this library.

Applications can bypass discovery entirely with:

    simulation.initialize("/absolute/path/to/mfr3duo_description/mjcf/scene.xml", options);

MFR3DUO_DESCRIPTION_PATH may point to scene.xml, the mfr3duo_description package directory, its installed share directory, or its installation prefix.

## Build

Install romujoco and make its CMake package visible, then build:

    cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH="$HOME/.local/romujoco"
    cmake --build build -j

If the description is installed in another prefix, append that prefix to CMAKE_PREFIX_PATH or specify:

    -DMFR3DUO_DESCRIPTION_SHARE_DIR=/path/to/share/mfr3duo_description

Run:

    ./build/mfr3duo_sim
    ./build/mfr3duo_sim --headless
    ./build/mfr3duo_sim --headless --no-cameras --steps 1000

Build the direct-control example with:

    cmake -S . -B build \
      -DMFR3DUO_MUJOCO_BUILD_EXAMPLES=ON \
      -DCMAKE_PREFIX_PATH="$HOME/.local/romujoco"
    cmake --build build -j
    ./build/mfr3duo_control_example

## Consume from another C++ project

After installation:

    find_package(mfr3duo_mujoco CONFIG REQUIRED)

    target_link_libraries(
      my_controller
      PRIVATE
        mfr3duo_mujoco::mfr3duo_mujoco)

The consumer-facing headers do not expose romujoco types.

## Tests

Run:

    ctest --test-dir build --output-on-failure

The configuration test checks the complete 40-component assembly. The simulation smoke test loads the real MFR3Duo scene, advances beyond one LiDAR period and verifies the robot-level state, IMU and both LiDAR APIs.

## Boundary

mfr3duo_description owns the robot model. romujoco owns generic MuJoCo runtime and generic device components. mfr3duo_mujoco owns MFR3Duo-specific assembly and the robot-level simulation API.

ROS 2, Python, network or other integrations may be implemented as optional adapters above this C++ API. They are not part of the core simulation architecture.