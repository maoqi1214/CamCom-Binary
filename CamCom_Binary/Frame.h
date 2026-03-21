#pragma once
#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <array>

// Forward declaration if needed elsewhere, though not necessary here
// class Frame108;

class Frame108 {
public:
    static constexpr int SIZE = 108;
    static constexpr int ANCHOR_SIZE = 7; // 定位点大小
    static constexpr int ROW_FRAME_ID_START = 7;
    static constexpr int ROW_CHECKSUM_START = 8;
    static constexpr int ROW_DATA_START = 9;
    static constexpr int DATA_BITS_PER_CELL = 2;
    static constexpr int FRAME_ID_BITS = 32;
    static constexpr int CHECKSUM_BITS = 32;
    static constexpr int DATA_CELL_COUNT = ((SIZE - ROW_DATA_START) * SIZE) - (ANCHOR_SIZE * ANCHOR_SIZE);
    static constexpr int DATA_BITS_CAPACITY = DATA_CELL_COUNT * DATA_BITS_PER_CELL;
    static constexpr int DATA_BYTES_CAPACITY = DATA_BITS_CAPACITY / 8;

    //构造函数
    Frame108();

    // --- 编码端接口 ---
    //设置定位点
    void setAnchorPoints();

    //设置帧编号
    void setFrameId(uint32_t id, int bits = FRAME_ID_BITS);

    //设置校验码
    void setChecksum(uint32_t crc, int bits = CHECKSUM_BITS);

    //设置数据区
    void setData(const std::vector<uint8_t>& data);

    // 一键编码：清空 -> 设置所有字段
    void encode(uint32_t frameId, const std::vector<uint8_t>& data, uint32_t crc);

    //导出为opencv图像
    cv::Mat toImage() const;

    // --- 解码端接口 ---
    // 自动处理：黑色定位点检测 -> 透视校正 -> 颜色解析
    // 返回 true 表示成功找到并校正，false 表示无法识别或校验失败
    static bool fromImageRobust(const cv::Mat& rawInput, Frame108& outFrame, int dataBits = DATA_BITS_CAPACITY);

    //从opencv图像解析(仅用于已校正的图像)
    static Frame108 fromImage(const cv::Mat& img);

    //获取帧编号
    uint32_t getFrameId(int bits = FRAME_ID_BITS) const;

    //获取校验码
    uint32_t getChecksum(int bits = CHECKSUM_BITS) const;

    //获取数据区
    std::vector<uint8_t> getData(int dataBits) const;

private:
    enum CellSymbol : uint8_t {
        White = 0,
        Red = 1,
        Green = 2,
        Blue = 3,
        Black = 4,
    };

    uint8_t matrix[SIZE][SIZE]; // 调色板索引，数据单元使用 White/Red/Green/Blue，定位点使用 Black

    // 内部辅助：寻找四个定位点角点
    static std::vector<cv::Point2f> findAnchorCorners(const cv::Mat& blackMask);
    static bool isAnchorCell(int x, int y);
    static bool isDataCell(int x, int y);
    static cv::Vec3b symbolToBgr(uint8_t symbol);
    static uint8_t bgrToSymbol(const cv::Vec3b& pixel);
};