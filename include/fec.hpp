// 声明帧内纠错编码与交织的接口。
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace camcom {

struct FecConfig {
    int rs_data_bytes = 239;
    int rs_parity_bytes = 16;
    int interleave_depth = 4;
};

const FecConfig& default_fec_config();

std::size_t fec_encoded_size_for_source_size(
    std::size_t source_size,
    const FecConfig& cfg = default_fec_config());

std::size_t fec_max_source_size_for_encoded_capacity(
    std::size_t encoded_capacity,
    const FecConfig& cfg = default_fec_config());

std::vector<uint8_t> fec_encode_payload(
    const std::vector<uint8_t>& source,
    std::size_t encoded_capacity,
    const FecConfig& cfg = default_fec_config());

bool fec_decode_payload(
    const std::vector<uint8_t>& encoded,
    std::size_t source_size,
    std::vector<uint8_t>& source_out,
    const FecConfig& cfg = default_fec_config());

} // namespace camcom
