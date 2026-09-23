#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace mfr3duo_mujoco {

/**
 * @brief Canonical MFR3Duo camera streams.
 */
enum class Camera : std::uint8_t {
    FrontColor,
    FrontDepth,
    RearColor,
    RearDepth,
    RightColor,
    RightDepth,
    LeftColor,
    LeftDepth,
    LeftWristColor,
    LeftWristDepth,
    RightWristColor,
    RightWristDepth,
    HeadZedLeft,
    HeadZedRight,
};

struct Image {
    std::uint64_t timestamp{0};
    std::string frame_id;
    std::uint32_t height{0};
    std::uint32_t width{0};
    std::string encoding;
    std::uint8_t is_bigendian{0};
    std::uint32_t step{0};
    std::vector<std::uint8_t> data;
};

struct CameraInfo {
    std::uint32_t height{0};
    std::uint32_t width{0};
    std::string distortion_model;
    std::vector<double> d;
    std::array<double, 9> k{};
    std::array<double, 9> r{};
    std::array<double, 12> p{};
    std::uint32_t binning_x{0};
    std::uint32_t binning_y{0};
};

/**
 * @brief One camera sample.
 *
 * RGB-only streams populate image, depth-only streams populate depth_image.
 */
struct CameraFrame {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    std::string frame_id;
    std::string optical_frame_id;
    Image image;
    Image depth_image;
    CameraInfo camera_info;
};

}  // namespace mfr3duo_mujoco
