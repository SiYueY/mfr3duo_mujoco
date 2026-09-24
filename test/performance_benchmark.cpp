#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "mfr3duo_mujoco/simulation.hpp"

namespace {

constexpr int kSteps = 200;
constexpr double kPhysicsPeriod = 0.001;

struct ModelDeleter {
    void operator()(mjModel* model) const noexcept { mj_deleteModel(model); }
};

struct DataDeleter {
    void operator()(mjData* data) const noexcept { mj_deleteData(data); }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

template <typename Step>
double measure(Step&& step) {
    const auto begin = std::chrono::steady_clock::now();
    for (int index = 0; index < kSteps; ++index) step();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(end - begin).count();
}

void print_result(const char* name, double wall_seconds) {
    const double simulation_seconds = kSteps * kPhysicsPeriod;
    const double step_us = wall_seconds * 1.0e6 / kSteps;
    const double rtf = simulation_seconds / wall_seconds;
    std::cout << std::left << std::setw(24) << name
              << " wall=" << std::fixed << std::setprecision(6) << wall_seconds
              << " s  step=" << std::setprecision(2) << step_us
              << " us  rtf=" << std::setprecision(3) << rtf << '\n';
}

bool reset_home(const mjModel& model, mjData& data) {
    const int key = mj_name2id(&model, mjOBJ_KEY, "home");
    if (key < 0) return false;
    mj_resetDataKeyframe(&model, &data, key);
    mj_forward(&model, &data);
    return true;
}

}  // namespace

int main() {
    char error[1024] = {};
    ModelPtr model(mj_loadXML(MFR3DUO_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
    if (!model) {
        std::cerr << "failed to load model: " << error << '\n';
        return EXIT_FAILURE;
    }

    DataPtr data(mj_makeData(model.get()));
    if (!data || !reset_home(*model, *data)) {
        std::cerr << "failed to initialize raw MuJoCo benchmark state\n";
        return EXIT_FAILURE;
    }

    const double raw_step = measure([&] { mj_step(model.get(), data.get()); });
    print_result("raw mj_step", raw_step);

    if (!reset_home(*model, *data)) return EXIT_FAILURE;
    const double forward_step = measure([&] {
        mj_forward(model.get(), data.get());
        mj_step(model.get(), data.get());
    });
    print_result("mj_forward + mj_step", forward_step);

    mfr3duo_mujoco::SimulationOptions options;
    options.viewer_enabled = false;
    options.cameras_enabled = false;
    options.lidars_enabled = false;
    options.imu_enabled = false;

    mfr3duo_mujoco::Simulation simulation;
    if (!simulation.initialize(MFR3DUO_TEST_SCENE_PATH, options)) {
        std::cerr << "failed to initialize mfr3duo_mujoco benchmark\n";
        return EXIT_FAILURE;
    }

    const auto begin = std::chrono::steady_clock::now();
    if (!simulation.step(kSteps)) {
        std::cerr << "mfr3duo_mujoco benchmark step failed\n";
        simulation.shutdown();
        return EXIT_FAILURE;
    }
    const auto end = std::chrono::steady_clock::now();
    const double framework =
        std::chrono::duration<double>(end - begin).count();
    print_result("romujoco full tick", framework);

    if (!simulation.shutdown()) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
