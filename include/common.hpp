// Shared constants, enums, and data structures for the encoder and decoder.
#pragma once

#include <cstddef>
#include <cstdint>

namespace camcom {

constexpr uint32_t MAGIC = 0x43414D43; // "CAMC"
constexpr uint8_t FORMAT_VERSION = 1;
constexpr int DEFAULT_FPS = 30;
constexpr int DEFAULT_CELL_SIZE = 20;
constexpr int FINDER_MARKER_CELLS = 7;

enum class ExitCode : int {
    Ok = 0,
    BadArgs = 1,
    IoError = 2,
    EncodingError = 3,
    DecodingError = 4,
};

enum class Encoding : uint8_t {
    Binary = 0,
    Gray4 = 1,
};

uint32_t crc32(const uint8_t* data, std::size_t length);

struct FrameHeader {
    uint32_t magic = 0;
    uint8_t version = 0;
    uint32_t frame_index = 0;
    uint32_t total_frames = 0;
    uint32_t payload_bytes = 0;
    uint32_t checksum = 0;
};

struct StreamHeader {
    uint32_t magic = 0;
    uint8_t version = 0;
    uint64_t total_data_bytes = 0;
    Encoding encoding = Encoding::Binary;
    uint32_t fps = 0;
    uint32_t cell_size = 0;
    uint32_t rs_nsym = 0;
    uint32_t payload_bytes_per_frame = 0;
    uint32_t cells_per_row = 0;
    uint32_t total_frames = 0;
};

} // namespace camcom
