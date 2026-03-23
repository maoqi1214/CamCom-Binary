// 声明可视化帧的渲染、矫正、采样与颜色校准接口。
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
    int reference_block_size = 2;
    cv::Scalar background_color = cv::Scalar(240, 240, 240);
    cv::Scalar colors[4] = {
        cv::Scalar(24, 24, 24),
        cv::Scalar(0, 184, 184),
        cv::Scalar(184, 184, 0),
        cv::Scalar(184, 0, 184),
    };
};

EncoderConfig make_default_encoder_config(int fps = DEFAULT_FPS);

void render_frame(
    cv::Mat& out,
    const std::vector<uint8_t>& payload,
    const EncoderConfig& cfg,
    int forced_rows = 0);
bool detect_frame_quad(
    const cv::Mat& frame,
    const EncoderConfig& cfg,
    std::array<cv::Point2f, 4>& out_quad);
bool rectify_frame_with_quad(
    const cv::Mat& frame,
    const std::array<cv::Point2f, 4>& quad,
    const EncoderConfig& cfg,
    cv::Mat& out_rectified);
bool sample_frame(const cv::Mat& frame, std::vector<uint8_t>& out_payload, EncoderConfig& cfg);
bool rectify_frame_geometry(const cv::Mat& frame, cv::Mat& out_rectified, const EncoderConfig& cfg);
double laplacian_variance(const cv::Mat& img);
cv::Vec3d compute_color_scale(
    const std::array<cv::Scalar, 4>& expected,
    const std::array<cv::Scalar, 4>& observed);

} // namespace camcom
