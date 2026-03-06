#pragma once

#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Project-wide constants
// ---------------------------------------------------------------------------

namespace camcom {

/// Magic bytes written at the start of every encoded stream.
constexpr uint32_t MAGIC = 0x43414D43; // "CAMC"

/// Current file-format version.
constexpr uint8_t FORMAT_VERSION = 1;

/// Default frames per second for encoded output video.
constexpr int DEFAULT_FPS = 30;

/// Size of the QR / data-matrix cell in pixels (width == height).
constexpr int DEFAULT_CELL_SIZE = 20;

// ---------------------------------------------------------------------------
// Shared types and enums
// ---------------------------------------------------------------------------

/// Exit codes used by both encoder and decoder.
enum class ExitCode : int {
    Ok              = 0,
    BadArgs         = 1,
    IoError         = 2,
    EncodingError   = 3,
    DecodingError   = 4,
};

/// Encoding scheme used to map bytes to visual symbols.
enum class Encoding : uint8_t {
    Binary = 0, ///< Pure black/white pixel blocks.
    Gray4  = 1, ///< 4-level grayscale (2 bits per cell).
    // TODO: add further encoding modes as needed
};

/// Compute a simple CRC-32 (ISO 3309 polynomial) over the supplied byte range.
uint32_t crc32(const uint8_t* data, std::size_t length);

/// Per-frame header embedded in the video stream.
struct FrameHeader {
    uint32_t magic;          ///< Must equal MAGIC.
    uint8_t  version;        ///< FORMAT_VERSION.
    uint32_t frame_index;    ///< 0-based frame number.
    uint32_t total_frames;   ///< Total number of data frames in this stream.
    uint32_t payload_bytes;  ///< Number of data bytes carried in this frame.
    uint32_t checksum;       ///< CRC-32 of the payload bytes in this frame.
};

/// Top-level stream header (written once at the start of the first frame).
struct StreamHeader {
    uint32_t magic;           ///< Must equal MAGIC.
    uint8_t  version;         ///< FORMAT_VERSION.
    uint64_t total_data_bytes;///< Original (unencoded) file size in bytes.
    Encoding encoding;        ///< Encoding scheme used.
    uint32_t fps;             ///< Frames per second of the output video.
    uint32_t cell_size;       ///< Visual cell size in pixels.
};

} // namespace camcom
