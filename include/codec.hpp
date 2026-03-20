// Visual frame rendering, sampling, and color calibration interfaces.
#pragma once

#include "common.hpp"

#include <array>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <vector>

namespace camcom {

struct EncoderConfig {
    int cell_size = DEFAULT_CELL_SIZE;
    int fps = 10;
    int payload_bytes_per_frame = 256;
    int cells_per_row = 32;
    int rs_nsym = 16;
    int reference_block_size = 2;
    cv::Scalar colors[4] = {
        cv::Scalar(0, 0, 0),
        cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 0, 255),
    };
};

void render_frame(cv::Mat& out, const std::vector<uint8_t>& payload, const EncoderConfig& cfg);
bool sample_frame(const cv::Mat& frame, std::vector<uint8_t>& out_payload, EncoderConfig& cfg);
double laplacian_variance(const cv::Mat& img);
cv::Vec3d compute_color_scale(
    const std::array<cv::Scalar, 4>& expected,
    const std::array<cv::Scalar, 4>& observed);

} // namespace camcom
