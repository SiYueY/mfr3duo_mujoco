#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include "keyboard.hpp"
#include "kinematics.hpp"
#include "mfr3duo_mujoco/simulation.hpp"
#include "teleop.hpp"

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) { stop_requested = 1; }

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "\n"
        << "Options:\n"
        << "  --headless        Disable the MuJoCo viewer.\n"
        << "  --scene PATH      Use an explicit MFR3Duo-compatible MJCF scene.\n"
        << "  --urdf PATH       Use an explicit MFR3Duo URDF for Pinocchio.\n"
        << "  --keyframe NAME   Initial MJCF keyframe (default: home).\n"
        << "  -h, --help        Show this help.\n";
}

std::string default_urdf_path(const std::string& scene) {
    const std::filesystem::path package =
        std::filesystem::path(scene).parent_path().parent_path();
    return (package / "urdf" / "mfr3duo.urdf").string();
}

}  // namespace

int main(int argc, char** argv) {
    using Clock = std::chrono::steady_clock;

    mfr3duo_mujoco::SimulationOptions simulation_options;
    simulation_options.cameras_enabled = false;
    simulation_options.lidars_enabled = false;
    simulation_options.imu_enabled = false;

    std::string scene;
    std::string urdf;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "-h" || argument == "--help") {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        }
        if (argument == "--headless") {
            simulation_options.viewer_enabled = false;
            continue;
        }
        if (argument == "--scene" && index + 1 < argc) {
            scene = argv[++index];
            continue;
        }
        if (argument == "--urdf" && index + 1 < argc) {
            urdf = argv[++index];
            continue;
        }
        if (argument == "--keyframe" && index + 1 < argc) {
            simulation_options.initial_keyframe = argv[++index];
            continue;
        }

        std::cerr << "unknown or incomplete option: " << argument << '\n';
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    try {
        if (scene.empty()) scene = mfr3duo_mujoco::scene_path();
        if (urdf.empty()) urdf = default_urdf_path(scene);

        if (!std::filesystem::is_regular_file(urdf)) {
            std::cerr << "MFR3Duo URDF not found: " << urdf << '\n';
            return EXIT_FAILURE;
        }

        mfr3duo_mujoco::Simulation simulation;
        if (!simulation.initialize(scene, simulation_options)) {
            std::cerr << "failed to initialize MFR3Duo simulation\n";
            return EXIT_FAILURE;
        }

        mfr3duo_mujoco::teleop::Kinematics kinematics;
        if (!kinematics.initialize(urdf)) {
            std::cerr
                << "failed to initialize Pinocchio kinematics from "
                << urdf << '\n';
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        mfr3duo_mujoco::teleop::Teleop teleop;
        if (!teleop.initialize(simulation, kinematics)) {
            std::cerr << "failed to initialize teleop controller\n";
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        mfr3duo_mujoco::teleop::Keyboard keyboard;
        if (!keyboard.open()) {
            std::cerr << "teleop requires an interactive terminal\n";
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        std::signal(SIGINT, request_stop);
        std::signal(SIGTERM, request_stop);

        if (!simulation.start()) {
            std::cerr << "failed to start MFR3Duo simulation\n";
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        teleop.print_help();
        teleop.print_status();

        constexpr auto period = std::chrono::milliseconds(10);
        auto previous = Clock::now();
        auto next = previous + period;
        bool failed = false;

        while (stop_requested == 0) {
            char key{};
            while (keyboard.read(key)) {
                bool exit_requested = false;
                if (!teleop.handle_key(
                        key, Clock::now(), exit_requested)) {
                    failed = true;
                    break;
                }
                if (exit_requested) {
                    stop_requested = 1;
                    break;
                }
            }
            if (failed || stop_requested != 0) break;

            const auto now = Clock::now();
            const double dt =
                std::chrono::duration<double>(now - previous).count();
            previous = now;

            if (!teleop.update(now, dt)) {
                std::cerr << "teleop control update failed\n";
                failed = true;
                break;
            }

            if (simulation.status() ==
                mfr3duo_mujoco::SimulationStatus::Error) {
                std::cerr
                    << "MFR3Duo simulation entered the error state\n";
                failed = true;
                break;
            }

            std::this_thread::sleep_until(next);
            next += period;
            if (next < Clock::now()) next = Clock::now() + period;
        }

        if (!failed) failed = !teleop.stop_motion();

        const auto status = simulation.status();
        if (status == mfr3duo_mujoco::SimulationStatus::Running ||
            status == mfr3duo_mujoco::SimulationStatus::Paused) {
            if (!simulation.stop()) failed = true;
        }

        if (!simulation.shutdown()) failed = true;
        return failed ? EXIT_FAILURE : EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "mfr3duo_teleop: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}