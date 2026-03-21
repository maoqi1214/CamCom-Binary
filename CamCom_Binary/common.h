#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

// 项目范围常量
namespace camcom {

    // 每个编码流开头的魔数
    // ASCII "CAMC"
    constexpr uint32_t MAGIC = 0x43414D43;

    // 当前文件格式版本
    constexpr uint8_t FORMAT_VERSION = 1;

    // 编码输出视频的默认帧率
    constexpr int DEFAULT_FPS = 30;

    // 数据矩阵单元格像素大小（宽==高）
    // 注意：这是默认值；实际单元格大小可在StreamHeader中指定
    constexpr int DEFAULT_CELL_SIZE = 20;

    // 共享类型和枚举
    // 编码器和解码器使用的退出码
    enum class ExitCode : int {
        Ok = 0,
        BadArgs = 1,
        IoError = 2,
        EncodingError = 3,
        DecodingError = 4,
    };

    // 将字节映射到视觉符号的编码方案
    enum class Encoding : uint8_t {
        Binary = 0,  // 纯黑白像素块（每单元1位）
        Gray4 = 1,   // 4级灰度（每单元2位）
        Color4 = 2,  // 4色调色板（白/红/绿/蓝，每单元2位）
    };

    // 嵌入视频流中的每帧头信息（概念上的，映射到Frame108结构）
    struct FrameHeader {
        uint32_t magic;          // 必须等于MAGIC
        uint8_t  version;        // FORMAT_VERSION
        uint32_t frame_index;    // 0基帧编号
        uint32_t total_frames;   // 此流中数据帧总数
        uint32_t payload_bytes;  // 此帧携带的数据字节数
        uint32_t checksum;       // 此帧中载荷字节的CRC-32校验和
    };

    // 顶层流头信息（传输整体的元数据）
    struct StreamHeader {
        uint32_t magic;           // 必须等于MAGIC
        uint8_t  version;         // FORMAT_VERSION
        uint64_t total_data_bytes; // 原始（未编码）文件大小（字节）
        Encoding encoding;        // 使用的编码方案
        uint32_t fps;             // 输出视频帧率
        uint32_t cell_size;       // 视觉单元格像素大小
    };

    // 函数声明
    // 计算指定字节范围的CRC-32（ISO 3309多项式）
    uint32_t crc32(const uint8_t* data, std::size_t length);

} // namespace camcom