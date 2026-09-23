#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int) { stop_requested = 1; }

void print_usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "\n"
        << "Options:\n"
        << "  --headless             Disable the interactive MuJoCo viewer.\n"
        << "  --no-cameras           Disable all RGB/depth camera components.\n"
        << "  --no-lidars            Disable both nanoScan3 components.\n"
        << "  --no-imu               Disable the base IMU component.\n"
        << "  --camera-width N       Render width for every camera (default: 1280).\n"
        << "  --camera-height N      Render height for every camera (default: 720).\n"
        << "  --keyframe NAME        Initial MJCF keyframe (default: home).\n"
        << "  --steps N              Execute exactly N headless steps and exit.\n"
        << "  -h, --help             Show this help.\n";
}

bool parse_positive_int(const char* value, int& out) {
    try {
        const long parsed = std::stol(value);
        if (parsed <= 0 || parsed > std::numeric_limits<int>::max()) {
            return false;
        }
        out = static_cast<int>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_positive_size(const char* value, std::size_t& out) {
    try {
        const unsigned long long parsed = std::stoull(value);
        if (parsed == 0 ||
            parsed > std::numeric_limits<std::size_t>::max()) {
            return false;
        }
        out = static_cast<std::size_t>(parsed);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    mfr3duo_mujoco::SimulationOptions options;
    std::size_t steps = 0;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "-h" || argument == "--help") {
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        }
        if (argument == "--headless") {
            options.viewer_enabled = false;
            continue;
        }
        if (argument == "--no-cameras") {
            options.cameras_enabled = false;
            continue;
        }
        if (argument == "--no-lidars") {
            options.lidars_enabled = false;
            continue;
        }
        if (argument == "--no-imu") {
            options.imu_enabled = false;
            continue;
        }
        if (argument == "--camera-width" && index + 1 < argc) {
            if (!parse_positive_int(argv[++index], options.camera_width)) {
                std::cerr << "invalid --camera-width value\n";
                return EXIT_FAILURE;
            }
            continue;
        }
        if (argument == "--camera-height" && index + 1 < argc) {
            if (!parse_positive_int(
                    argv[++index], options.camera_height)) {
                std::cerr << "invalid --camera-height value\n";
                return EXIT_FAILURE;
            }
            continue;
        }
        if (argument == "--keyframe" && index + 1 < argc) {
            options.initial_keyframe = argv[++index];
            continue;
        }
        if (argument == "--steps" && index + 1 < argc) {
            if (!parse_positive_size(argv[++index], steps)) {
                std::cerr << "invalid --steps value\n";
                return EXIT_FAILURE;
            }
            options.viewer_enabled = false;
            continue;
        }
        std::cerr << "unknown or incomplete option: " << argument << '\n';
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    try {
        mfr3duo_mujoco::Simulation simulation;
        if (!simulation.initialize(options)) {
            std::cerr << "failed to initialize MFR3Duo simulation\n";
            return EXIT_FAILURE;
        }

        if (steps != 0U) {
            const bool succeeded = simulation.step(steps);
            const bool shutdown = simulation.shutdown();
            return succeeded && shutdown
                       ? EXIT_SUCCESS
                       : EXIT_FAILURE;
        }

        std::signal(SIGINT, request_stop);
        std::signal(SIGTERM, request_stop);
        if (!simulation.start()) {
            std::cerr << "failed to start MFR3Duo simulation\n";
            simulation.shutdown();
            return EXIT_FAILURE;
        }

        while (stop_requested == 0) {
            const auto status = simulation.status();
            if (status == mfr3duo_mujoco::SimulationStatus::Error) {
                std::cerr
                    << "MFR3Duo simulation entered the error state\n";
                simulation.shutdown();
                return EXIT_FAILURE;
            }
            if (status == mfr3duo_mujoco::SimulationStatus::Stopped) {
                break;
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(20));
        }

        const auto status = simulation.status();
        if (status == mfr3duo_mujoco::SimulationStatus::Running ||
            status == mfr3duo_mujoco::SimulationStatus::Paused) {
            if (!simulation.stop()) {
                simulation.shutdown();
                return EXIT_FAILURE;
            }
        }
        return simulation.shutdown()
                   ? EXIT_SUCCESS
                   : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr
            << "mfr3duo_mujoco: "
            << error.what()
            << '\n';
        return EXIT_FAILURE;
    }
}