#include "common.hpp"

#include <mutex>
#include <stdexcept>
#include <string>
#include <cstring>

namespace camcom {

// ---------------------------------------------------------------------------
// CRC-32 implementation (ISO 3309 / ITU-T V.42 polynomial)
// ---------------------------------------------------------------------------

static uint32_t crc32_table[256];
static std::once_flag crc32_init_flag;

static void build_crc32_table() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 1u) ? (0xEDB88320u ^ (crc >> 1)) : (crc >> 1);
        }
        crc32_table[i] = crc;
    }
}

uint32_t crc32(const uint8_t* data, std::size_t length) {
    std::call_once(crc32_init_flag, build_crc32_table);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < length; ++i) {
        crc = crc32_table[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// Little-endian serialisation helpers (private)
// ---------------------------------------------------------------------------

static void write_le32(std::vector<uint8_t>& out, uint32_t v) {
    out.push_back(static_cast<uint8_t>(v));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v >> 16));
    out.push_back(static_cast<uint8_t>(v >> 24));
}

static void write_le64(std::vector<uint8_t>& out, uint64_t v) {
    for (int i = 0; i < 8; ++i)
        out.push_back(static_cast<uint8_t>(v >> (i * 8)));
}

static uint32_t read_le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

static uint64_t read_le64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
        v |= static_cast<uint64_t>(p[i]) << (i * 8);
    return v;
}

// ---------------------------------------------------------------------------
// StreamHeader serialisation
// ---------------------------------------------------------------------------

std::vector<uint8_t> serialize_stream_header(const StreamHeader& hdr) {
    std::vector<uint8_t> out;
    out.reserve(STREAM_HEADER_SIZE);
    write_le32(out, hdr.magic);                              // 4 bytes
    out.push_back(hdr.version);                              // 1 byte
    write_le64(out, hdr.total_data_bytes);                   // 8 bytes
    out.push_back(static_cast<uint8_t>(hdr.encoding));       // 1 byte
    write_le32(out, hdr.fps);                                // 4 bytes
    write_le32(out, hdr.cell_size);                          // 4 bytes
    return out; // total = 22 bytes
}

bool deserialize_stream_header(const uint8_t* data, std::size_t len, StreamHeader& out) {
    if (len < STREAM_HEADER_SIZE) return false;
    out.magic            = read_le32(data);       data += 4;
    out.version          = *data++;
    out.total_data_bytes = read_le64(data);       data += 8;
    out.encoding         = static_cast<Encoding>(*data++);
    out.fps              = read_le32(data);       data += 4;
    out.cell_size        = read_le32(data);
    return true;
}

// ---------------------------------------------------------------------------
// FrameHeader serialisation
// ---------------------------------------------------------------------------

std::vector<uint8_t> serialize_frame_header(const FrameHeader& hdr) {
    std::vector<uint8_t> out;
    out.reserve(FRAME_HEADER_SIZE);
    write_le32(out, hdr.magic);          // 4 bytes
    out.push_back(hdr.version);          // 1 byte
    write_le32(out, hdr.frame_index);    // 4 bytes
    write_le32(out, hdr.total_frames);   // 4 bytes
    write_le32(out, hdr.payload_bytes);  // 4 bytes
    write_le32(out, hdr.checksum);       // 4 bytes
    return out; // total = 21 bytes
}

bool deserialize_frame_header(const uint8_t* data, std::size_t len, FrameHeader& out) {
    if (len < FRAME_HEADER_SIZE) return false;
    out.magic         = read_le32(data); data += 4;
    out.version       = *data++;
    out.frame_index   = read_le32(data); data += 4;
    out.total_frames  = read_le32(data); data += 4;
    out.payload_bytes = read_le32(data); data += 4;
    out.checksum      = read_le32(data);
    return true;
}

} // namespace camcom
