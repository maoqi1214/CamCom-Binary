#include "common.hpp"

#include <mutex>
#include <stdexcept>
#include <string>

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

} // namespace camcom
