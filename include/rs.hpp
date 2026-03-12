#pragma once

#include <vector>
#include <cstdint>

namespace camcom { namespace rs {

// Initialize GF(256) tables (primitive 0x11d). Called automatically on first use.
void init_tables();

// Encode message (msg) with nsym parity bytes. Returns parity bytes (length nsym).
std::vector<uint8_t> encode(const std::vector<uint8_t>& msg, int nsym);

// Decode codeword in-place (msg+parity). nsym is number of parity bytes.
// Returns true if decoding succeeded (corrected or already valid), false otherwise.
bool decode(std::vector<uint8_t>& codeword, int nsym);

}} // namespace camcom::rs
