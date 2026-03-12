#pragma once

#include <opencv2/opencv.hpp>
#include "common.hpp"

#include <cstdint>
#include <vector>
#include <array>

namespace camcom {

struct EncoderConfig {
    int cell_size = DEFAULT_CELL_SIZE; // 单元格像素尺寸
    int fps = 10; // 保守默认帧率，尽量避免滚动快门影响
    int payload_bytes_per_frame = 256; // 每帧载荷字节数（可配置）
    int cells_per_row = 32; // 每行数据单元格数量
    int rs_nsym = 16; // Reed-Solomon 冗余字节数
    int reference_block_size = 2; // 参考色块尺寸（按单元格计，正方形）
    // 颜色采用 BGR 顺序（OpenCV 默认使用 BGR）
    cv::Scalar colors[4] = {
        cv::Scalar(0,0,0),      // 00 -> 黑色
        cv::Scalar(255,0,0),    // 01 -> 蓝色
        cv::Scalar(0,255,0),    // 10 -> 绿色
        cv::Scalar(0,0,255)     // 11 -> 红色
    };
};

// 将单帧数据渲染为图像（数据单元格）。
// 图像中包含四角定位标记和数据区域。
void render_frame(cv::Mat& out, const std::vector<uint8_t>& payload, const EncoderConfig& cfg);

// 对输入帧（可为已矫正的正视图）进行网格采样，恢复载荷字节。
// 成功返回 true。
bool sample_frame(const cv::Mat& warped, std::vector<uint8_t>& out_payload, const EncoderConfig& cfg);

// 返回拉普拉斯方差（清晰度指标），值越低表示越模糊。
double laplacian_variance(const cv::Mat& img);

// 简单颜色标定：根据“观测参考色”和“期望参考色”
// 计算每个通道的缩放系数，用于匹配前校正采样均值。
cv::Vec3d compute_color_scale(const std::array<cv::Scalar,4>& expected, const std::array<cv::Scalar,4>& observed);

} // namespace camcom
