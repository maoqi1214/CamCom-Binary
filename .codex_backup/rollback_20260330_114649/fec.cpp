// 实现帧内 RS 纠错与交织逻辑。
#include "fec.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <vector>

namespace camcom {
namespace {

class Gf256 {
public:
    Gf256() {
        int value = 1;
        for (int i = 0; i < 255; ++i) {
            exp_[i] = static_cast<uint8_t>(value);
            log_[static_cast<size_t>(value)] = static_cast<uint8_t>(i);
            value <<= 1;
            if ((value & 0x100) != 0) {
                value ^= 0x11D;
            }
        }
        for (int i = 255; i < 512; ++i) {
            exp_[i] = exp_[i - 255];
        }
        log_[0] = 0;
    }

    uint8_t mul(uint8_t a, uint8_t b) const {
        if (a == 0 || b == 0) {
            return 0;
        }
        return exp_[static_cast<size_t>(log_[a]) + log_[b]];
    }

    uint8_t div(uint8_t a, uint8_t b) const {
        if (b == 0) {
            throw std::runtime_error("GF256 divide by zero");
        }
        if (a == 0) {
            return 0;
        }
        int diff = static_cast<int>(log_[a]) - static_cast<int>(log_[b]);
        if (diff < 0) {
            diff += 255;
        }
        return exp_[static_cast<size_t>(diff)];
    }

    uint8_t inverse(uint8_t a) const {
        if (a == 0) {
            throw std::runtime_error("GF256 inverse of zero");
        }
        return exp_[255 - log_[a]];
    }

    uint8_t pow(uint8_t a, int power) const {
        if (power == 0) {
            return 1;
        }
        if (a == 0) {
            return 0;
        }
        int exponent = static_cast<int>(log_[a]) * power;
        exponent %= 255;
        if (exponent < 0) {
            exponent += 255;
        }
        return exp_[static_cast<size_t>(exponent)];
    }

    uint8_t alpha_pow(int power) const {
        int exponent = power % 255;
        if (exponent < 0) {
            exponent += 255;
        }
        return exp_[static_cast<size_t>(exponent)];
    }

private:
    std::array<uint8_t, 512> exp_{};
    std::array<uint8_t, 256> log_{};
};

const Gf256& gf256() {
    static const Gf256 field;
    return field;
}

std::vector<uint8_t> poly_add(const std::vector<uint8_t>& lhs, const std::vector<uint8_t>& rhs) {
    const std::size_t n = std::max(lhs.size(), rhs.size());
    std::vector<uint8_t> out(n, 0);

    for (std::size_t i = 0; i < lhs.size(); ++i) {
        out[n - lhs.size() + i] ^= lhs[i];
    }
    for (std::size_t i = 0; i < rhs.size(); ++i) {
        out[n - rhs.size() + i] ^= rhs[i];
    }
    return out;
}

std::vector<uint8_t> poly_scale(const std::vector<uint8_t>& poly, uint8_t x) {
    std::vector<uint8_t> out(poly.size(), 0);
    for (std::size_t i = 0; i < poly.size(); ++i) {
        out[i] = gf256().mul(poly[i], x);
    }
    return out;
}

std::vector<uint8_t> poly_mul(const std::vector<uint8_t>& lhs, const std::vector<uint8_t>& rhs) {
    std::vector<uint8_t> out(lhs.size() + rhs.size() - 1, 0);
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        for (std::size_t j = 0; j < rhs.size(); ++j) {
            out[i + j] ^= gf256().mul(lhs[i], rhs[j]);
        }
    }
    return out;
}

uint8_t poly_eval(const std::vector<uint8_t>& poly, uint8_t x) {
    uint8_t y = 0;
    for (const uint8_t coef : poly) {
        y = gf256().mul(y, x) ^ coef;
    }
    return y;
}

std::pair<std::vector<uint8_t>, std::vector<uint8_t>> poly_div(
    const std::vector<uint8_t>& dividend,
    const std::vector<uint8_t>& divisor) {
    std::vector<uint8_t> msg_out = dividend;
    const std::size_t normalizer_len = divisor.size() - 1;
    for (std::size_t i = 0; i + divisor.size() <= dividend.size(); ++i) {
        const uint8_t coef = msg_out[i];
        if (coef == 0) {
            continue;
        }
        for (std::size_t j = 1; j < divisor.size(); ++j) {
            if (divisor[j] != 0) {
                msg_out[i + j] ^= gf256().mul(divisor[j], coef);
            }
        }
    }

    std::vector<uint8_t> quotient(
        msg_out.begin(),
        msg_out.begin() + static_cast<std::ptrdiff_t>(dividend.size() - normalizer_len));
    std::vector<uint8_t> remainder(
        msg_out.end() - static_cast<std::ptrdiff_t>(normalizer_len),
        msg_out.end());
    return {quotient, remainder};
}

std::vector<uint8_t> make_generator_poly(int nsym) {
    std::vector<uint8_t> generator = {1};
    for (int i = 0; i < nsym; ++i) {
        generator = poly_mul(generator, {1, gf256().alpha_pow(i)});
    }
    return generator;
}

const std::vector<uint8_t>& generator_poly(int nsym) {
    if (nsym <= 0 || nsym >= 255) {
        throw std::runtime_error("RS parity size must be in [1, 254]");
    }

    static std::vector<std::vector<uint8_t>> cache(255);
    auto& generator = cache[static_cast<std::size_t>(nsym)];
    if (generator.empty()) {
        generator = make_generator_poly(nsym);
    }
    return generator;
}

std::vector<uint8_t> rs_calc_syndromes(const std::vector<uint8_t>& msg, int nsym) {
    std::vector<uint8_t> syndromes(static_cast<std::size_t>(nsym) + 1, 0);
    for (int i = 0; i < nsym; ++i) {
        syndromes[static_cast<std::size_t>(i) + 1] = poly_eval(msg, gf256().alpha_pow(i));
    }
    return syndromes;
}

bool syndromes_all_zero(const std::vector<uint8_t>& syndromes) {
    return std::all_of(
        syndromes.begin() + 1,
        syndromes.end(),
        [](uint8_t value) { return value == 0; });
}

std::vector<uint8_t> rs_find_error_locator(const std::vector<uint8_t>& syndromes, int nsym) {
    std::vector<uint8_t> err_loc = {1};
    std::vector<uint8_t> old_loc = {1};

    for (int i = 0; i < nsym; ++i) {
        uint8_t delta = syndromes[static_cast<std::size_t>(i) + 1];
        for (std::size_t j = 1; j < err_loc.size(); ++j) {
            delta ^= gf256().mul(
                err_loc[err_loc.size() - 1 - j],
                syndromes[static_cast<std::size_t>(i) + 1 - j]);
        }

        old_loc.push_back(0);
        if (delta == 0) {
            continue;
        }

        if (old_loc.size() > err_loc.size()) {
            std::vector<uint8_t> new_loc = poly_scale(old_loc, delta);
            old_loc = poly_scale(err_loc, gf256().inverse(delta));
            err_loc = std::move(new_loc);
        }
        err_loc = poly_add(err_loc, poly_scale(old_loc, delta));
    }

    while (!err_loc.empty() && err_loc.front() == 0) {
        err_loc.erase(err_loc.begin());
    }

    const int errs = static_cast<int>(err_loc.size()) - 1;
    if (errs * 2 > nsym) {
        return {};
    }
    return err_loc;
}

std::vector<int> rs_find_errors(const std::vector<uint8_t>& err_loc_reversed, int message_len) {
    const int errs = static_cast<int>(err_loc_reversed.size()) - 1;
    std::vector<int> err_pos;
    err_pos.reserve(static_cast<std::size_t>(errs));

    for (int i = 0; i < message_len; ++i) {
        if (poly_eval(err_loc_reversed, gf256().alpha_pow(i)) == 0) {
            err_pos.push_back(message_len - 1 - i);
        }
    }

    if (static_cast<int>(err_pos.size()) != errs) {
        return {};
    }
    return err_pos;
}

std::vector<uint8_t> rs_find_errata_locator(const std::vector<int>& coef_pos) {
    std::vector<uint8_t> err_loc = {1};
    for (const int pos : coef_pos) {
        err_loc = poly_mul(err_loc, poly_add({1}, {gf256().alpha_pow(pos), 0}));
    }
    return err_loc;
}

std::vector<uint8_t> rs_find_error_evaluator(
    const std::vector<uint8_t>& syndromes_reversed,
    const std::vector<uint8_t>& err_loc,
    int err_loc_degree) {
    const auto [_, remainder] = poly_div(
        poly_mul(syndromes_reversed, err_loc),
        std::vector<uint8_t>(static_cast<std::size_t>(err_loc_degree) + 2, 0));
    return remainder;
}

bool rs_correct_errata(
    std::vector<uint8_t>& msg,
    const std::vector<uint8_t>& syndromes,
    const std::vector<int>& err_pos) {
    std::vector<int> coef_pos;
    coef_pos.reserve(err_pos.size());
    for (const int pos : err_pos) {
        coef_pos.push_back(static_cast<int>(msg.size()) - 1 - pos);
    }

    const std::vector<uint8_t> err_loc = rs_find_errata_locator(coef_pos);
    std::vector<uint8_t> syndromes_reversed(syndromes.rbegin(), syndromes.rend());
    std::vector<uint8_t> err_eval =
        rs_find_error_evaluator(syndromes_reversed, err_loc, static_cast<int>(err_loc.size()) - 1);
    std::reverse(err_eval.begin(), err_eval.end());

    std::vector<uint8_t> X;
    X.reserve(coef_pos.size());
    for (const int pos : coef_pos) {
        X.push_back(gf256().alpha_pow(-(255 - pos)));
    }

    std::vector<uint8_t> E(msg.size(), 0);
    for (std::size_t i = 0; i < X.size(); ++i) {
        const uint8_t Xi = X[i];
        const uint8_t Xi_inv = gf256().inverse(Xi);

        uint8_t err_loc_prime = 1;
        for (std::size_t j = 0; j < X.size(); ++j) {
            if (j == i) {
                continue;
            }
            err_loc_prime = gf256().mul(
                err_loc_prime,
                static_cast<uint8_t>(1 ^ gf256().mul(Xi_inv, X[j])));
        }
        if (err_loc_prime == 0) {
            return false;
        }

        std::vector<uint8_t> err_eval_reversed(err_eval.rbegin(), err_eval.rend());
        uint8_t y = poly_eval(err_eval_reversed, Xi_inv);
        y = gf256().mul(Xi, y);
        const uint8_t magnitude = gf256().div(y, err_loc_prime);
        E[static_cast<std::size_t>(err_pos[i])] = magnitude;
    }

    for (std::size_t i = 0; i < msg.size(); ++i) {
        msg[i] ^= E[i];
    }
    return true;
}

bool rs_decode_block_full(std::vector<uint8_t>& full_codeword, int nsym) {
    std::vector<uint8_t> syndromes = rs_calc_syndromes(full_codeword, nsym);
    if (syndromes_all_zero(syndromes)) {
        return true;
    }

    std::vector<uint8_t> err_loc = rs_find_error_locator(syndromes, nsym);
    if (err_loc.empty()) {
        return false;
    }

    std::vector<uint8_t> err_loc_reversed(err_loc.rbegin(), err_loc.rend());
    std::vector<int> err_pos = rs_find_errors(err_loc_reversed, static_cast<int>(full_codeword.size()));
    if (err_pos.empty()) {
        return false;
    }

    if (!rs_correct_errata(full_codeword, syndromes, err_pos)) {
        return false;
    }
    return syndromes_all_zero(rs_calc_syndromes(full_codeword, nsym));
}

std::vector<uint8_t> rs_encode_shortened_block(
    const std::vector<uint8_t>& data,
    const FecConfig& cfg) {
    if (data.size() > static_cast<std::size_t>(cfg.rs_data_bytes)) {
        throw std::runtime_error("RS source block is too large");
    }

    const std::size_t prefix_zeros =
        static_cast<std::size_t>(cfg.rs_data_bytes) - data.size();
    std::vector<uint8_t> padded_data(prefix_zeros, 0);
    padded_data.insert(padded_data.end(), data.begin(), data.end());

    const std::vector<uint8_t>& generator = generator_poly(cfg.rs_parity_bytes);
    std::vector<uint8_t> msg_out = padded_data;
    msg_out.insert(msg_out.end(), static_cast<std::size_t>(cfg.rs_parity_bytes), 0);

    for (std::size_t i = 0; i < padded_data.size(); ++i) {
        const uint8_t coef = msg_out[i];
        if (coef == 0) {
            continue;
        }
        for (std::size_t j = 1; j < generator.size(); ++j) {
            msg_out[i + j] ^= gf256().mul(generator[j], coef);
        }
    }

    std::vector<uint8_t> out = data;
    out.insert(
        out.end(),
        msg_out.end() - static_cast<std::ptrdiff_t>(cfg.rs_parity_bytes),
        msg_out.end());
    return out;
}

bool rs_decode_shortened_block(
    const std::vector<uint8_t>& encoded_block,
    std::size_t source_size,
    std::vector<uint8_t>& decoded_block,
    const FecConfig& cfg) {
    if (source_size > static_cast<std::size_t>(cfg.rs_data_bytes) ||
        encoded_block.size() != source_size + static_cast<std::size_t>(cfg.rs_parity_bytes)) {
        return false;
    }

    const std::size_t prefix_zeros =
        static_cast<std::size_t>(cfg.rs_data_bytes) - source_size;
    std::vector<uint8_t> full_codeword(prefix_zeros, 0);
    full_codeword.insert(full_codeword.end(), encoded_block.begin(), encoded_block.end());
    if (!rs_decode_block_full(full_codeword, cfg.rs_parity_bytes)) {
        return false;
    }

    decoded_block.assign(
        full_codeword.begin() + static_cast<std::ptrdiff_t>(prefix_zeros),
        full_codeword.begin() + static_cast<std::ptrdiff_t>(prefix_zeros + source_size));
    return true;
}

std::vector<uint8_t> interleave_bytes(const std::vector<uint8_t>& data, int depth) {
    if (depth <= 1 || data.empty()) {
        return data;
    }

    std::vector<std::vector<uint8_t>> buckets(static_cast<std::size_t>(depth));
    for (std::size_t i = 0; i < data.size(); ++i) {
        buckets[i % static_cast<std::size_t>(depth)].push_back(data[i]);
    }

    std::vector<uint8_t> out;
    out.reserve(data.size());
    for (const auto& bucket : buckets) {
        out.insert(out.end(), bucket.begin(), bucket.end());
    }
    return out;
}

std::vector<uint8_t> deinterleave_bytes(const std::vector<uint8_t>& data, int depth) {
    if (depth <= 1 || data.empty()) {
        return data;
    }

    std::vector<uint8_t> out(data.size(), 0);
    const std::size_t depth_u = static_cast<std::size_t>(depth);
    const std::size_t base = data.size() / depth_u;
    const std::size_t extra = data.size() % depth_u;

    std::size_t offset = 0;
    for (std::size_t bucket_index = 0; bucket_index < depth_u; ++bucket_index) {
        const std::size_t bucket_size = base + (bucket_index < extra ? 1 : 0);
        for (std::size_t j = 0; j < bucket_size; ++j) {
            out[bucket_index + j * depth_u] = data[offset + j];
        }
        offset += bucket_size;
    }
    return out;
}

} // namespace

const FecConfig& default_fec_config() {
    static const FecConfig cfg;
    return cfg;
}

std::size_t fec_encoded_size_for_source_size(std::size_t source_size, const FecConfig& cfg) {
    if (source_size == 0) {
        return 0;
    }

    const std::size_t full_blocks =
        source_size / static_cast<std::size_t>(cfg.rs_data_bytes);
    const std::size_t remainder =
        source_size % static_cast<std::size_t>(cfg.rs_data_bytes);

    std::size_t encoded = full_blocks * static_cast<std::size_t>(cfg.rs_data_bytes + cfg.rs_parity_bytes);
    if (remainder > 0) {
        encoded += remainder + static_cast<std::size_t>(cfg.rs_parity_bytes);
    }
    return encoded;
}

std::size_t fec_max_source_size_for_encoded_capacity(
    std::size_t encoded_capacity,
    const FecConfig& cfg) {
    const std::size_t full_code_bytes =
        static_cast<std::size_t>(cfg.rs_data_bytes + cfg.rs_parity_bytes);
    const std::size_t full_blocks = encoded_capacity / full_code_bytes;
    const std::size_t remainder = encoded_capacity % full_code_bytes;

    std::size_t source = full_blocks * static_cast<std::size_t>(cfg.rs_data_bytes);
    if (remainder > static_cast<std::size_t>(cfg.rs_parity_bytes)) {
        source += remainder - static_cast<std::size_t>(cfg.rs_parity_bytes);
    }
    return source;
}

std::vector<uint8_t> fec_encode_payload(
    const std::vector<uint8_t>& source,
    std::size_t encoded_capacity,
    const FecConfig& cfg) {
    if (source.size() > fec_max_source_size_for_encoded_capacity(encoded_capacity, cfg)) {
        throw std::runtime_error("Source payload exceeds FEC frame capacity");
    }

    std::vector<uint8_t> encoded;
    encoded.reserve(fec_encoded_size_for_source_size(source.size(), cfg));

    for (std::size_t offset = 0; offset < source.size();) {
        const std::size_t block_size = std::min(
            static_cast<std::size_t>(cfg.rs_data_bytes),
            source.size() - offset);
        const std::vector<uint8_t> block(
            source.begin() + static_cast<std::ptrdiff_t>(offset),
            source.begin() + static_cast<std::ptrdiff_t>(offset + block_size));
        const std::vector<uint8_t> encoded_block = rs_encode_shortened_block(block, cfg);
        encoded.insert(encoded.end(), encoded_block.begin(), encoded_block.end());
        offset += block_size;
    }

    return interleave_bytes(encoded, cfg.interleave_depth);
}

bool fec_decode_payload(
    const std::vector<uint8_t>& encoded,
    std::size_t source_size,
    std::vector<uint8_t>& source_out,
    const FecConfig& cfg) {
    if (encoded.size() != fec_encoded_size_for_source_size(source_size, cfg)) {
        return false;
    }

    const std::vector<uint8_t> deinterleaved = deinterleave_bytes(encoded, cfg.interleave_depth);
    std::vector<uint8_t> decoded;
    decoded.reserve(source_size);

    std::size_t source_offset = 0;
    std::size_t encoded_offset = 0;
    while (source_offset < source_size) {
        const std::size_t block_source_size = std::min(
            static_cast<std::size_t>(cfg.rs_data_bytes),
            source_size - source_offset);
        const std::size_t block_encoded_size =
            block_source_size + static_cast<std::size_t>(cfg.rs_parity_bytes);

        const std::vector<uint8_t> encoded_block(
            deinterleaved.begin() + static_cast<std::ptrdiff_t>(encoded_offset),
            deinterleaved.begin() + static_cast<std::ptrdiff_t>(encoded_offset + block_encoded_size));

        std::vector<uint8_t> decoded_block;
        if (!rs_decode_shortened_block(encoded_block, block_source_size, decoded_block, cfg)) {
            return false;
        }
        decoded.insert(decoded.end(), decoded_block.begin(), decoded_block.end());

        source_offset += block_source_size;
        encoded_offset += block_encoded_size;
    }

    source_out = std::move(decoded);
    return true;
}

} // namespace camcom
