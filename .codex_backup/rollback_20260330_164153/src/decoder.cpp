// 实现 decode 主程序，包括帧解析、恢复与二进制重组逻辑。
#include "codec.hpp"
#include "common.hpp"
#include "fec.hpp"
#include "io.hpp"
#include "tracker.hpp"

#include <algorithm>
#include <deque>
#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace camcom;
namespace {

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.mp4> <output.bin> <validity.bin> [reference.bin]\n"
        << "Example: " << argv0 << " input.mp4 output.bin validity.bin input.bin\n";
}

void print_progress_bar(size_t current, size_t total) {
    constexpr size_t kBarWidth = 40;
    const size_t safe_total = std::max<size_t>(total, 1);
    const size_t clamped_current = std::min(current, safe_total);
    const size_t filled = (clamped_current * kBarWidth) / safe_total;
    const double percent = 100.0 * static_cast<double>(clamped_current) / static_cast<double>(safe_total);

    std::cout << "\rDecoding frames [";
    for (size_t i = 0; i < kBarWidth; ++i) {
        std::cout << (i < filled ? '#' : '-');
    }
    std::cout << "] "
        << std::setw(6) << std::fixed << std::setprecision(2) << percent << "% ("
        << clamped_current << "/" << total << ")" << std::flush;
}

void print_recovery_progress_bar(size_t current, size_t total) {
    constexpr size_t kBarWidth = 32;
    const size_t safe_total = std::max<size_t>(total, 1);
    const size_t clamped_current = std::min(current, safe_total);
    const size_t filled = (clamped_current * kBarWidth) / safe_total;
    const double percent = 100.0 * static_cast<double>(clamped_current) / static_cast<double>(safe_total);

    std::cout << "\rRecovering missing frames [";
    for (size_t i = 0; i < kBarWidth; ++i) {
        std::cout << (i < filled ? '#' : '-');
    }
    std::cout << "] "
              << std::setw(6) << std::fixed << std::setprecision(2) << percent << "% ("
              << clamped_current << "/" << total << ")" << std::flush;
}

std::vector<uint8_t> majority_vote_samples(
    const std::vector<uint8_t>& a,
    const std::vector<uint8_t>& b,
    const std::vector<uint8_t>& c) {
    const size_t n = std::min({a.size(), b.size(), c.size()});
    std::vector<uint8_t> out(n, 0);
    for (size_t i = 0; i < n; ++i) {
        uint8_t value = 0;
        for (int bit = 0; bit < 8; ++bit) {
            const int ones =
                ((a[i] >> bit) & 1u) +
                ((b[i] >> bit) & 1u) +
                ((c[i] >> bit) & 1u);
            if (ones >= 2) {
                value |= static_cast<uint8_t>(1u << bit);
            }
        }
        out[i] = value;
    }
    return out;
}

std::vector<uint8_t> majority_vote_samples(const std::deque<std::vector<uint8_t>>& samples) {
    if (samples.empty()) {
        return {};
    }

    size_t n = samples.front().size();
    for (const auto& sample : samples) {
        n = std::min(n, sample.size());
    }

    std::vector<uint8_t> out(n, 0);
    const int threshold = static_cast<int>(samples.size() / 2) + 1;
    for (size_t i = 0; i < n; ++i) {
        uint8_t value = 0;
        for (int bit = 0; bit < 8; ++bit) {
            int ones = 0;
            for (const auto& sample : samples) {
                ones += (sample[i] >> bit) & 1u;
            }
            if (ones >= threshold) {
                value |= static_cast<uint8_t>(1u << bit);
            }
        }
        out[i] = value;
    }
    return out;
}

std::vector<uint8_t> plurality_vote_samples(const std::vector<std::vector<uint8_t>>& samples) {
    if (samples.empty()) {
        return {};
    }

    size_t n = samples.front().size();
    for (const auto& sample : samples) {
        n = std::min(n, sample.size());
    }

    std::vector<uint8_t> out(n, 0);
    for (size_t i = 0; i < n; ++i) {
        std::array<int, 256> counts{};
        int best_count = -1;
        uint8_t best_value = 0;
        for (const auto& sample : samples) {
            const uint8_t candidate = sample[i];
            const int count = ++counts[candidate];
            if (count > best_count) {
                best_count = count;
                best_value = candidate;
            }
        }
        out[i] = best_value;
    }
    return out;
}

uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
        (static_cast<uint32_t>(p[1]) << 8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24);
}

uint64_t read_u64_le(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return value;
}

size_t max_source_payload_bytes_per_frame(const EncoderConfig& cfg) {
    return fec_max_source_size_for_encoded_capacity(
        static_cast<size_t>(cfg.payload_bytes_per_frame));
}

int header_hamming_distance_at(const std::vector<uint8_t>& sample, size_t off) {
    constexpr uint8_t target[5] = {0x43, 0x4D, 0x41, 0x43, 0x01};
    if (off + 5 > sample.size()) {
        return 9999;
    }

    int distance = 0;
    for (size_t i = 0; i < 5; ++i) {
        uint8_t diff = static_cast<uint8_t>(sample[off + i] ^ target[i]);
        while (diff != 0) {
            distance += diff & 1u;
            diff >>= 1;
        }
    }
    return distance;
}

bool is_black_frame(const cv::Mat& frame) {
    const cv::Scalar mean = cv::mean(frame);
    return mean[0] < 5 && mean[1] < 5 && mean[2] < 5;
}

cv::Mat make_frame_signature(const cv::Mat& frame) {
    cv::Mat color;
    if (frame.channels() == 3) {
        color = frame;
    } else {
        cv::cvtColor(frame, color, cv::COLOR_GRAY2BGR);
    }

    cv::Mat small;
    cv::resize(color, small, cv::Size(96, 54), 0.0, 0.0, cv::INTER_AREA);
    cv::GaussianBlur(small, small, cv::Size(5, 5), 0.0);
    return small;
}

double signature_difference(const cv::Mat& a, const cv::Mat& b) {
    if (a.empty() || b.empty() || a.size() != b.size()) {
        return std::numeric_limits<double>::infinity();
    }

    cv::Mat diff;
    cv::absdiff(a, b, diff);
    const cv::Scalar mean = cv::mean(diff);
    return (mean[0] + mean[1] + mean[2]) / 3.0;
}

bool try_parse_bootstrap(const std::vector<uint8_t>& sample, EncoderConfig& cfg) {
    constexpr size_t kBootstrapBytes = 4 + 1 + 5 * 4;
    if (sample.size() < kBootstrapBytes) {
        return false;
    }

    for (size_t off = 0; off + kBootstrapBytes <= sample.size(); ++off) {
        const uint8_t* p = sample.data() + static_cast<std::ptrdiff_t>(off);
        const uint32_t magic = read_u32_le(p); p += 4;
        const uint8_t version = *p; p += 1;
        if (magic != MAGIC || version != FORMAT_VERSION) {
            continue;
        }

        const uint32_t cell_size = read_u32_le(p); p += 4;
        const uint32_t cells_per_row = read_u32_le(p); p += 4;
        const uint32_t payload_per = read_u32_le(p); p += 4;
        const uint32_t fps = read_u32_le(p); p += 4;
        const uint32_t reference_block_size = read_u32_le(p); p += 4;

        const bool sane =
            cell_size >= 8 && cell_size <= 64 &&
            cells_per_row >= 16 && cells_per_row <= 256 &&
            payload_per >= 64 && payload_per <= 16384 &&
            fps >= 1 && fps <= 60 &&
            reference_block_size >= 1 && reference_block_size <= 8;
        if (!sane) {
            continue;
        }

        cfg.cell_size = static_cast<int>(cell_size);
        cfg.fps = static_cast<int>(fps);
        cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
        cfg.cells_per_row = static_cast<int>(cells_per_row);
        cfg.reference_block_size = static_cast<int>(reference_block_size);
        return true;
    }

    return false;
}

bool try_parse_stream_header(
    const std::vector<uint8_t>& sample,
    EncoderConfig& cfg,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes) {
    constexpr size_t kStreamHeaderBytes = 4 + 1 + 8 + 1 + 5 * 4;
    if (sample.size() < kStreamHeaderBytes) {
        return false;
    }

    for (size_t off = 0; off + kStreamHeaderBytes <= sample.size(); ++off) {
        const uint8_t* p = sample.data() + static_cast<std::ptrdiff_t>(off);
        const uint32_t magic = read_u32_le(p); p += 4;
        const uint8_t version = *p; p += 1;
        if (magic != MAGIC || version != FORMAT_VERSION) {
            continue;
        }

        const uint64_t total_data = read_u64_le(p); p += 8;
        p += 1; // encoding
        const uint32_t fps = read_u32_le(p); p += 4;
        const uint32_t cell_size = read_u32_le(p); p += 4;
        const uint32_t payload_per = read_u32_le(p); p += 4;
        const uint32_t cells_per_row = read_u32_le(p); p += 4;
        const uint32_t total_frames_hdr = read_u32_le(p); p += 4;

        const bool sane =
            cell_size >= 8 && cell_size <= 64 &&
            payload_per >= 64 && payload_per <= 16384 &&
            cells_per_row >= 16 && cells_per_row <= 256 &&
            total_frames_hdr >= 1 && total_frames_hdr <= 100000 &&
            fps >= 1 && fps <= 60 &&
            total_data > 0;
        if (!sane) {
            continue;
        }

        cfg.cell_size = static_cast<int>(cell_size);
        cfg.fps = static_cast<int>(fps);
        cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
        cfg.cells_per_row = static_cast<int>(cells_per_row);
        expected_total_frames = total_frames_hdr;
        expected_total_bytes = total_data;
        return true;
    }

    return false;
}

bool try_parse_data_frame(
    const std::vector<uint8_t>& sample,
    const EncoderConfig& cfg,
    FrameHeader& hdr,
    std::vector<uint8_t>& payload_out) {
    constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
    if (sample.size() < kFrameHeaderBytes) {
        return false;
    }

    const size_t source_payload_limit = max_source_payload_bytes_per_frame(cfg);
    for (size_t off = 0; off + kFrameHeaderBytes <= sample.size(); ++off) {
        const uint8_t* raw = sample.data() + static_cast<std::ptrdiff_t>(off);
        FrameHeader local{};
        local.magic = read_u32_le(raw);
        local.version = raw[4];
        if (local.magic != MAGIC || local.version != FORMAT_VERSION) {
            continue;
        }

        local.frame_index = read_u32_le(raw + 5);
        local.total_frames = read_u32_le(raw + 9);
        local.payload_bytes = read_u32_le(raw + 13);
        local.checksum = read_u32_le(raw + 17);

        if (local.total_frames == 0 || local.frame_index >= local.total_frames) {
            continue;
        }
        if (local.payload_bytes == 0 ||
            local.payload_bytes > static_cast<uint32_t>(source_payload_limit)) {
            continue;
        }

        const size_t source_payload_len = static_cast<size_t>(local.payload_bytes);
        const size_t encoded_payload_len =
            fec_encoded_size_for_source_size(source_payload_len);
        const size_t raw_payload_off = off + kFrameHeaderBytes;
        const size_t raw_payload_end = raw_payload_off + encoded_payload_len;
        if (raw_payload_end <= sample.size()) {
            const std::vector<uint8_t> encoded_payload(
                sample.begin() + static_cast<std::ptrdiff_t>(raw_payload_off),
                sample.begin() + static_cast<std::ptrdiff_t>(raw_payload_end));
            std::vector<uint8_t> decoded_payload;
            if (fec_decode_payload(encoded_payload, source_payload_len, decoded_payload) &&
                decoded_payload.size() == source_payload_len &&
                crc32(decoded_payload.data(), decoded_payload.size()) == local.checksum) {
                hdr = local;
                payload_out = std::move(decoded_payload);
                return true;
            }
        }
    }

    return false;
}

bool try_parse_raw_frame_header(
    const std::vector<uint8_t>& sample,
    const EncoderConfig& cfg,
    FrameHeader& hdr,
    size_t& header_offset) {
    constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
    if (sample.size() < kFrameHeaderBytes) {
        return false;
    }

    for (size_t off = 0; off + kFrameHeaderBytes <= sample.size(); ++off) {
        const uint8_t* raw = sample.data() + static_cast<std::ptrdiff_t>(off);
        if (read_u32_le(raw) != MAGIC || raw[4] != FORMAT_VERSION) {
            continue;
        }

        FrameHeader local{};
        local.magic = MAGIC;
        local.version = FORMAT_VERSION;
        local.frame_index = read_u32_le(raw + 5);
        local.total_frames = read_u32_le(raw + 9);
        local.payload_bytes = read_u32_le(raw + 13);
        local.checksum = read_u32_le(raw + 17);

        if (local.total_frames == 0 || local.frame_index >= local.total_frames) {
            continue;
        }
        if (local.payload_bytes == 0 ||
            local.payload_bytes > static_cast<uint32_t>(max_source_payload_bytes_per_frame(cfg))) {
            continue;
        }

        hdr = local;
        header_offset = off;
        return true;
    }

    return false;
}

bool find_best_raw_header_candidate(
    const std::vector<uint8_t>& sample,
    const EncoderConfig& cfg,
    FrameHeader& hdr,
    size_t& header_offset,
    int& header_distance) {
    constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
    if (sample.size() < kFrameHeaderBytes) {
        return false;
    }

    bool found = false;
    int best_distance = 9999;
    FrameHeader best_hdr{};
    size_t best_offset = 0;
    for (size_t off = 0; off + kFrameHeaderBytes <= sample.size(); ++off) {
        const int distance = header_hamming_distance_at(sample, off);
        if (distance > 14) {
            continue;
        }

        FrameHeader local{};
        local.magic = MAGIC;
        local.version = FORMAT_VERSION;
        local.frame_index = read_u32_le(sample.data() + static_cast<std::ptrdiff_t>(off + 5));
        local.total_frames = read_u32_le(sample.data() + static_cast<std::ptrdiff_t>(off + 9));
        local.payload_bytes = read_u32_le(sample.data() + static_cast<std::ptrdiff_t>(off + 13));
        local.checksum = read_u32_le(sample.data() + static_cast<std::ptrdiff_t>(off + 17));

        const bool sane =
            local.total_frames > 0 &&
            local.total_frames <= 100000 &&
            local.frame_index < local.total_frames &&
            local.payload_bytes > 0 &&
            local.payload_bytes <= static_cast<uint32_t>(max_source_payload_bytes_per_frame(cfg));
        if (!sane) {
            continue;
        }

        if (!found || distance < best_distance) {
            found = true;
            best_distance = distance;
            best_hdr = local;
            best_offset = off;
        }
    }

    if (!found) {
        return false;
    }

    hdr = best_hdr;
    header_offset = best_offset;
    header_distance = best_distance;
    return true;
}

struct RawFrameSampleGroup {
    FrameHeader hdr{};
    std::vector<std::vector<uint8_t>> samples;
};

struct TrackingDecodeState {
    QuadTracker quad_tracker;
    int groups_since_measurement = 0;
};

bool refresh_tracker_measurement(
    const cv::Mat& frame,
    const EncoderConfig& cfg,
    TrackingDecodeState& tracking) {
    std::array<cv::Point2f, 4> measured_quad{};
    if (!detect_frame_quad(frame, cfg, measured_quad)) {
        return false;
    }

    if (!tracking.quad_tracker.is_initialized()) {
        tracking.quad_tracker.init(measured_quad);
    } else {
        tracking.quad_tracker.update(measured_quad);
    }
    tracking.groups_since_measurement = 0;
    return true;
}

bool sample_frame_with_quad_hint(
    const cv::Mat& frame,
    EncoderConfig& cfg,
    const std::array<cv::Point2f, 4>* quad_hint,
    std::vector<uint8_t>& sample_out) {
    if (quad_hint != nullptr) {
        cv::Mat rectified;
        EncoderConfig hinted_cfg = cfg;
        if (rectify_frame_with_quad(frame, *quad_hint, hinted_cfg, rectified) &&
            sample_frame(rectified, sample_out, hinted_cfg) &&
            !sample_out.empty()) {
            cfg = hinted_cfg;
            return true;
        }
    }

    EncoderConfig fallback_cfg = cfg;
    if (!sample_frame(frame, sample_out, fallback_cfg) || sample_out.empty()) {
        return false;
    }
    cfg = fallback_cfg;
    return true;
}

void recover_grouped_frames(
    const std::unordered_map<uint32_t, RawFrameSampleGroup>& raw_frame_samples,
    const EncoderConfig& cfg,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer) {
    for (const auto& [frame_index, group] : raw_frame_samples) {
        if (frames_buffer.find(frame_index) != frames_buffer.end() || group.samples.empty()) {
            continue;
        }

        FrameHeader hdr{};
        std::vector<uint8_t> payload;
        bool recovered = false;

        for (const auto& sample : group.samples) {
            if (try_parse_data_frame(sample, cfg, hdr, payload)) {
                recovered = true;
                break;
            }
        }

        if (!recovered && group.samples.size() >= 2) {
            std::deque<std::vector<uint8_t>> all_samples(group.samples.begin(), group.samples.end());
            std::vector<std::vector<uint8_t>> voted_candidates;
            voted_candidates.push_back(majority_vote_samples(all_samples));
            voted_candidates.push_back(plurality_vote_samples(group.samples));

            if (group.samples.size() >= 3) {
                voted_candidates.push_back(majority_vote_samples(
                    group.samples[0],
                    group.samples[1],
                    group.samples[2]));
                voted_candidates.push_back(majority_vote_samples(
                    group.samples[group.samples.size() - 3],
                    group.samples[group.samples.size() - 2],
                    group.samples[group.samples.size() - 1]));
                voted_candidates.push_back(plurality_vote_samples({
                    group.samples[0],
                    group.samples[1],
                    group.samples[2],
                }));
                voted_candidates.push_back(plurality_vote_samples({
                    group.samples[group.samples.size() - 3],
                    group.samples[group.samples.size() - 2],
                    group.samples[group.samples.size() - 1],
                }));
            }

            for (const auto& voted : voted_candidates) {
                if (try_parse_data_frame(voted, cfg, hdr, payload)) {
                    recovered = true;
                    break;
                }
            }
        }

        if (!recovered && group.hdr.payload_bytes > 0) {
            const size_t payload_len = static_cast<size_t>(group.hdr.payload_bytes);
            const size_t encoded_payload_len = fec_encoded_size_for_source_size(payload_len);
            std::vector<std::vector<uint8_t>> payload_samples;
            payload_samples.reserve(group.samples.size());
            std::vector<uint32_t> checksum_samples;
            checksum_samples.reserve(group.samples.size());

            for (const auto& sample : group.samples) {
                if (sample.size() < 21 + encoded_payload_len) {
                    continue;
                }
                payload_samples.emplace_back(
                    sample.begin() + 21,
                    sample.begin() + 21 + static_cast<std::ptrdiff_t>(encoded_payload_len));
                checksum_samples.push_back(read_u32_le(sample.data() + 17));
            }

            if (payload_samples.size() >= 2) {
                auto plurality_checksum = [&]() {
                    uint32_t best_value = group.hdr.checksum;
                    int best_count = -1;
                    for (const uint32_t candidate : checksum_samples) {
                        int count = 0;
                        for (const uint32_t other : checksum_samples) {
                            count += other == candidate ? 1 : 0;
                        }
                        if (count > best_count) {
                            best_count = count;
                            best_value = candidate;
                        }
                    }
                    return best_value;
                };

                std::deque<std::vector<uint8_t>> payload_deque(payload_samples.begin(), payload_samples.end());
                std::vector<std::vector<uint8_t>> payload_candidates;
                payload_candidates.push_back(majority_vote_samples(payload_deque));
                payload_candidates.push_back(plurality_vote_samples(payload_samples));
                if (payload_samples.size() >= 3) {
                    payload_candidates.push_back(majority_vote_samples(
                        payload_samples[0],
                        payload_samples[1],
                        payload_samples[2]));
                    payload_candidates.push_back(majority_vote_samples(
                        payload_samples[payload_samples.size() - 3],
                        payload_samples[payload_samples.size() - 2],
                        payload_samples[payload_samples.size() - 1]));
                    payload_candidates.push_back(plurality_vote_samples({
                        payload_samples[0],
                        payload_samples[1],
                        payload_samples[2],
                    }));
                    payload_candidates.push_back(plurality_vote_samples({
                        payload_samples[payload_samples.size() - 3],
                        payload_samples[payload_samples.size() - 2],
                        payload_samples[payload_samples.size() - 1],
                    }));
                }

                const std::array<uint32_t, 2> checksum_candidates = {
                    group.hdr.checksum,
                    plurality_checksum(),
                };
                for (const uint32_t checksum : checksum_candidates) {
                    for (const auto& candidate_payload : payload_candidates) {
                        std::vector<uint8_t> decoded_candidate;
                        if (candidate_payload.size() == encoded_payload_len &&
                            fec_decode_payload(candidate_payload, payload_len, decoded_candidate) &&
                            decoded_candidate.size() == payload_len &&
                            crc32(decoded_candidate.data(), decoded_candidate.size()) == checksum) {
                            hdr = group.hdr;
                            hdr.checksum = checksum;
                            payload = std::move(decoded_candidate);
                            recovered = true;
                            break;
                        }
                    }
                    if (recovered) {
                        break;
                    }
                }
            }

            if (!recovered &&
                group.hdr.total_frames > 0 &&
                group.hdr.frame_index + 1 == group.hdr.total_frames &&
                payload_len < max_source_payload_bytes_per_frame(cfg) &&
                payload_samples.size() >= 2) {
                const std::vector<uint8_t> candidate_payload = plurality_vote_samples(payload_samples);
                std::vector<uint8_t> decoded_candidate;
                if (candidate_payload.size() == encoded_payload_len &&
                    fec_decode_payload(candidate_payload, payload_len, decoded_candidate) &&
                    decoded_candidate.size() == payload_len) {
                    hdr = group.hdr;
                    payload = std::move(decoded_candidate);
                    recovered = true;
                }
            }

        }

        if (recovered) {
            frames_buffer.emplace(frame_index, std::move(payload));
        }
    }
}

uint64_t infer_total_bytes_from_frames(
    const std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer,
    uint32_t expected_total_frames,
    const EncoderConfig& cfg) {
    if (expected_total_frames == 0 || frames_buffer.empty()) {
        return 0;
    }

    const auto last_it = frames_buffer.find(expected_total_frames - 1);
    if (last_it == frames_buffer.end() || last_it->second.empty()) {
        return 0;
    }

    return static_cast<uint64_t>(expected_total_frames - 1) *
        static_cast<uint64_t>(max_source_payload_bytes_per_frame(cfg)) +
        static_cast<uint64_t>(last_it->second.size());
}

bool process_sample_candidate(
    const std::vector<uint8_t>& sample_candidate,
    EncoderConfig& cfg,
    EncoderConfig& candidate_cfg,
    bool& have_bootstrap,
    bool& have_stream_header,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer,
    std::unordered_map<uint32_t, RawFrameSampleGroup>& raw_frame_samples,
    std::unordered_map<uint64_t, int>& frame_payload_votes,
    std::unordered_map<uint32_t, uint32_t>& frame_best_payload_crc) {
    FrameHeader raw_hdr{};
    size_t header_offset = 0;
    int header_distance = 0;
    bool have_group_header = false;
    if (try_parse_raw_frame_header(sample_candidate, candidate_cfg, raw_hdr, header_offset)) {
        have_group_header = true;
    } else if (find_best_raw_header_candidate(
                   sample_candidate,
                   candidate_cfg,
                   raw_hdr,
                   header_offset,
                   header_distance)) {
        have_group_header = true;
    }

    if (have_group_header) {
        auto& group = raw_frame_samples[raw_hdr.frame_index];
        if (group.samples.empty()) {
            group.hdr = raw_hdr;
        }
        if (group.hdr.total_frames == raw_hdr.total_frames &&
            group.hdr.payload_bytes == raw_hdr.payload_bytes) {
            group.samples.emplace_back(
                sample_candidate.begin() + static_cast<std::ptrdiff_t>(header_offset),
                sample_candidate.end());
        }
    }

    bool matched = false;
    if (!have_bootstrap) {
        if (try_parse_bootstrap(sample_candidate, candidate_cfg)) {
            have_bootstrap = true;
            matched = true;
        }
    }
    if (!have_stream_header) {
        if (try_parse_stream_header(
                sample_candidate,
                candidate_cfg,
                expected_total_frames,
                expected_total_bytes)) {
            have_stream_header = true;
            matched = true;
        }
    }

    FrameHeader hdr{};
    std::vector<uint8_t> payload;
    if (try_parse_data_frame(sample_candidate, candidate_cfg, hdr, payload)) {
        if (expected_total_frames == 0 && hdr.total_frames > 0) {
            expected_total_frames = hdr.total_frames;
        }

        const uint32_t payload_crc = crc32(payload.data(), payload.size());
        const uint64_t vote_key =
            (static_cast<uint64_t>(hdr.frame_index) << 32) | payload_crc;
        const int votes = ++frame_payload_votes[vote_key];

        int best_votes = 0;
        if (const auto best_it = frame_best_payload_crc.find(hdr.frame_index);
            best_it != frame_best_payload_crc.end()) {
            const uint64_t best_key =
                (static_cast<uint64_t>(hdr.frame_index) << 32) | best_it->second;
            if (const auto vote_it = frame_payload_votes.find(best_key);
                vote_it != frame_payload_votes.end()) {
                best_votes = vote_it->second;
            }
        }

        if (votes >= best_votes) {
            frame_best_payload_crc[hdr.frame_index] = payload_crc;
            frames_buffer[hdr.frame_index] = std::move(payload);
        }
        matched = true;
    }

    if (matched) {
        cfg = candidate_cfg;
    }
    return matched;
}

void process_frame(
    const cv::Mat& frame,
    EncoderConfig& cfg,
    bool& have_bootstrap,
    bool& have_stream_header,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer,
    std::unordered_map<int, std::deque<std::vector<uint8_t>>>& sample_histories,
    std::unordered_map<uint32_t, RawFrameSampleGroup>& raw_frame_samples,
    std::unordered_map<uint64_t, int>& frame_payload_votes,
    std::unordered_map<uint32_t, uint32_t>& frame_best_payload_crc) {
    if (frame.empty() || is_black_frame(frame)) {
        return;
    }

    auto try_with_config = [&](EncoderConfig candidate_cfg) {
        std::vector<uint8_t> sample;
        if (!sample_frame(frame, sample, candidate_cfg)) {
            return false;
        }

        const int history_key =
            candidate_cfg.cells_per_row * 10000 +
            candidate_cfg.payload_bytes_per_frame;
        auto& history = sample_histories[history_key];
        history.push_back(sample);
        while (history.size() > 5) {
            history.pop_front();
        }

        std::vector<std::vector<uint8_t>> samples_to_try;
        if (history.size() >= 5) {
            std::deque<std::vector<uint8_t>> last5(
                history.end() - 5,
                history.end());
            samples_to_try.push_back(majority_vote_samples(last5));
            samples_to_try.push_back(plurality_vote_samples(std::vector<std::vector<uint8_t>>(last5.begin(), last5.end())));
        }
        if (history.size() >= 3) {
            const size_t n = history.size();
            samples_to_try.push_back(majority_vote_samples(
                history[n - 3],
                history[n - 2],
                history[n - 1]));
            samples_to_try.push_back(plurality_vote_samples({
                history[n - 3],
                history[n - 2],
                history[n - 1],
            }));
        }
        samples_to_try.push_back(sample);

        for (const auto& sample_candidate : samples_to_try) {
            if (process_sample_candidate(
                    sample_candidate,
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
        }
        return false;
    };

    if (try_with_config(cfg)) {
        return;
    }

}

void process_frame_group(
    std::vector<cv::Mat>& group_frames,
    EncoderConfig& cfg,
    bool& have_bootstrap,
    bool& have_stream_header,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer,
    std::unordered_map<uint32_t, RawFrameSampleGroup>& raw_frame_samples,
    TrackingDecodeState& tracking,
    std::unordered_map<uint64_t, int>& frame_payload_votes,
    std::unordered_map<uint32_t, uint32_t>& frame_best_payload_crc) {
    if (group_frames.empty()) {
        return;
    }

    std::vector<std::pair<double, size_t>> ranked_frames;
    ranked_frames.reserve(group_frames.size());
    for (size_t i = 0; i < group_frames.size(); ++i) {
        if (group_frames[i].empty() || is_black_frame(group_frames[i])) {
            continue;
        }
        ranked_frames.push_back({laplacian_variance(group_frames[i]), i});
    }
    if (ranked_frames.empty()) {
        group_frames.clear();
        return;
    }

    std::sort(ranked_frames.begin(), ranked_frames.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.first > rhs.first;
    });
    if (ranked_frames.size() > 12) {
        ranked_frames.resize(12);
    }

    std::array<cv::Point2f, 4> quad_hint{};
    const bool have_quad_hint = tracking.quad_tracker.is_initialized();
    if (have_quad_hint) {
        quad_hint = tracking.quad_tracker.predict();
    }
    const bool should_refresh_measurement =
        !tracking.quad_tracker.is_initialized() || tracking.groups_since_measurement >= 6;

    std::vector<std::vector<uint8_t>> samples;
    samples.reserve(ranked_frames.size());
    EncoderConfig candidate_cfg = cfg;
    size_t attempted_frames = 0;
    int best_successful_frame_index = -1;
    bool matched = false;
    const std::array<size_t, 3> stage_limits = {3u, 6u, 12u};

    auto process_current_samples = [&]() {
        if (samples.empty()) {
            return false;
        }

        const std::deque<std::vector<uint8_t>> sample_deque(samples.begin(), samples.end());
        if (samples.size() >= 3) {
            if (process_sample_candidate(
                    majority_vote_samples(sample_deque),
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
            if (process_sample_candidate(
                    plurality_vote_samples(samples),
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
            if (process_sample_candidate(
                    majority_vote_samples(samples[0], samples[1], samples[2]),
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
        }
        if (samples.size() >= 2) {
            if (process_sample_candidate(
                    majority_vote_samples(sample_deque),
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
            if (process_sample_candidate(
                    plurality_vote_samples(samples),
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
        }
        for (const auto& sample : samples) {
            if (process_sample_candidate(
                    sample,
                    cfg,
                    candidate_cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    frame_payload_votes,
                    frame_best_payload_crc)) {
                return true;
            }
        }
        return false;
    };

    for (const size_t limit : stage_limits) {
        const size_t stage_end = std::min(limit, ranked_frames.size());
        for (; attempted_frames < stage_end; ++attempted_frames) {
            const size_t frame_index = ranked_frames[attempted_frames].second;
            std::vector<uint8_t> sample;
            if (sample_frame_with_quad_hint(
                    group_frames[frame_index],
                    candidate_cfg,
                    have_quad_hint ? &quad_hint : nullptr,
                    sample) &&
                !sample.empty()) {
                if (best_successful_frame_index < 0) {
                    best_successful_frame_index = static_cast<int>(frame_index);
                }
                samples.push_back(std::move(sample));
            }
        }

        matched = process_current_samples();
        if (matched || attempted_frames == ranked_frames.size()) {
            break;
        }
    }

    if (best_successful_frame_index >= 0) {
        if (should_refresh_measurement || matched) {
            if (!refresh_tracker_measurement(
                    group_frames[static_cast<size_t>(best_successful_frame_index)],
                    candidate_cfg,
                    tracking) &&
                tracking.quad_tracker.is_initialized()) {
                ++tracking.groups_since_measurement;
            }
        } else if (tracking.quad_tracker.is_initialized()) {
            ++tracking.groups_since_measurement;
        }
    } else if (tracking.quad_tracker.is_initialized()) {
        ++tracking.groups_since_measurement;
    }

    group_frames.clear();
}

bool decode_frames_from_video(
    const std::string& video_path,
    EncoderConfig& cfg,
    bool& have_bootstrap,
    bool& have_stream_header,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer) {
    cv::VideoCapture cap(video_path, cv::CAP_ANY);
    if (!cap.isOpened()) {
        return false;
    }

    const double input_fps = cap.get(cv::CAP_PROP_FPS);
    const bool screen_recording_mode = input_fps > 35.0;
    const double reported_total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    const size_t total_frames =
        reported_total_frames > 0.0 ? static_cast<size_t>(std::llround(reported_total_frames)) : 0;

    std::cout << "Reading video frames with OpenCV VideoCapture...\n";
    if (screen_recording_mode) {
        std::cout << "Input FPS " << std::fixed << std::setprecision(2) << input_fps
                  << ", grouping near-identical captures before decoding.\n";
    } else {
        std::cout << "Input FPS " << std::fixed << std::setprecision(2) << input_fps
                  << ", decoding frames directly.\n";
    }
    size_t processed_frames = 0;
    size_t last_reported_frames = static_cast<size_t>(-1);
    std::unordered_map<int, std::deque<std::vector<uint8_t>>> sample_histories;
    std::unordered_map<uint32_t, RawFrameSampleGroup> raw_frame_samples;
    std::unordered_map<uint64_t, int> frame_payload_votes;
    std::unordered_map<uint32_t, uint32_t> frame_best_payload_crc;
    TrackingDecodeState tracking;
    if (total_frames > 0) {
        std::cout << "Backend reported " << total_frames << " frame(s).\n";
    }

    std::vector<cv::Mat> screen_group_frames;
    cv::Mat previous_signature;
    constexpr double kSameFrameThreshold = 2.0;

    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) {
            continue;
        }

        ++processed_frames;
        if (screen_recording_mode) {
            const cv::Mat signature = make_frame_signature(frame);
            if (!screen_group_frames.empty() &&
                signature_difference(previous_signature, signature) > kSameFrameThreshold) {
                process_frame_group(
                    screen_group_frames,
                    cfg,
                    have_bootstrap,
                    have_stream_header,
                    expected_total_frames,
                    expected_total_bytes,
                    frames_buffer,
                    raw_frame_samples,
                    tracking,
                    frame_payload_votes,
                    frame_best_payload_crc);
            }
            previous_signature = signature;
            screen_group_frames.push_back(frame.clone());
        } else {
            process_frame(
                frame,
                cfg,
                have_bootstrap,
                have_stream_header,
                expected_total_frames,
                expected_total_bytes,
                frames_buffer,
                sample_histories,
                raw_frame_samples,
                frame_payload_votes,
                frame_best_payload_crc);
        }

        if (total_frames > 0 &&
            (processed_frames == total_frames ||
             processed_frames == 1 ||
             processed_frames - last_reported_frames >= 10)) {
            print_progress_bar(processed_frames, total_frames);
            last_reported_frames = processed_frames;
        }
    }

    if (screen_recording_mode) {
        process_frame_group(
            screen_group_frames,
            cfg,
            have_bootstrap,
            have_stream_header,
            expected_total_frames,
            expected_total_bytes,
            frames_buffer,
            raw_frame_samples,
            tracking,
            frame_payload_votes,
            frame_best_payload_crc);
    }

    if (total_frames > 0) {
        print_progress_bar(processed_frames, total_frames);
        std::cout << "\n";
    }

    size_t grouped_header_count = 0;
    size_t multi_sample_group_count = 0;
    size_t max_group_samples = 0;
    for (const auto& [_, group] : raw_frame_samples) {
        ++grouped_header_count;
        max_group_samples = std::max(max_group_samples, group.samples.size());
        if (group.samples.size() >= 2) {
            ++multi_sample_group_count;
        }
    }
    std::cout << "raw header groups : " << grouped_header_count << "\n";
    std::cout << "multi-sample groups: " << multi_sample_group_count << "\n";
    std::cout << "max group samples : " << max_group_samples << "\n";
    const size_t grouped_recovery_before = frames_buffer.size();
    std::cout << "recovering grouped samples...\n";
    recover_grouped_frames(raw_frame_samples, cfg, frames_buffer);
    std::cout << "recovered by grouped samples: "
              << (frames_buffer.size() - grouped_recovery_before) << "\n";
    return true;
}

void recover_missing_frames_from_video_windows(
    const std::string& video_path,
    const EncoderConfig& cfg,
    uint32_t expected_total_frames,
    std::unordered_map<uint32_t, std::vector<uint8_t>>& frames_buffer) {
    if (expected_total_frames == 0 || frames_buffer.size() >= expected_total_frames || cfg.fps <= 0) {
        return;
    }

    std::vector<uint32_t> missing_indices;
    missing_indices.reserve(expected_total_frames - static_cast<uint32_t>(frames_buffer.size()));
    for (uint32_t i = 0; i < expected_total_frames; ++i) {
        if (frames_buffer.find(i) == frames_buffer.end()) {
            missing_indices.push_back(i);
        }
    }
    if (missing_indices.empty()) {
        return;
    }
    std::cout << "recovering missing frames in local windows...\n";
    std::cout << "missing before recovery: " << missing_indices.size()
              << "/" << expected_total_frames << "\n";

    cv::VideoCapture cap(video_path, cv::CAP_ANY);
    if (!cap.isOpened()) {
        return;
    }

    const double capture_fps = std::max(1.0, cap.get(cv::CAP_PROP_FPS));
    const double capture_ratio = capture_fps / static_cast<double>(cfg.fps);
    const int total_capture_frames = std::max(0, static_cast<int>(std::llround(cap.get(cv::CAP_PROP_FRAME_COUNT))));
    constexpr int kLeadInFrames = 6;
    constexpr int kWindowMarginFrames = 24;
    constexpr size_t kGlobalScanMinMissing = 12;

    cv::Mat frame;
    size_t recovered_count = 0;
    size_t processed_missing = 0;
    size_t last_reported = static_cast<size_t>(-1);

    if (missing_indices.size() >= kGlobalScanMinMissing) {
        std::cout << "large missing set detected, switching to full-video recovery scan...\n";
        std::unordered_set<uint32_t> missing_set(missing_indices.begin(), missing_indices.end());
        if (!cap.set(cv::CAP_PROP_POS_FRAMES, 0)) {
            return;
        }

        size_t scanned_frames = 0;
        size_t last_scan_reported = static_cast<size_t>(-1);
        while (!missing_set.empty() && cap.read(frame)) {
            if (frame.empty()) {
                continue;
            }
            ++scanned_frames;

            EncoderConfig candidate_cfg = cfg;
            std::vector<uint8_t> sample;
            if (!sample_frame(frame, sample, candidate_cfg)) {
                continue;
            }

            FrameHeader hdr{};
            std::vector<uint8_t> payload;
            if (!try_parse_data_frame(sample, candidate_cfg, hdr, payload)) {
                continue;
            }

            if (missing_set.erase(hdr.frame_index) > 0) {
                frames_buffer[hdr.frame_index] = std::move(payload);
                ++recovered_count;
            }

            if (total_capture_frames > 0 &&
                (scanned_frames == static_cast<size_t>(total_capture_frames) ||
                 scanned_frames == 1 ||
                 scanned_frames - last_scan_reported >= 20)) {
                print_recovery_progress_bar(scanned_frames, static_cast<size_t>(total_capture_frames));
                last_scan_reported = scanned_frames;
            }
        }

        if (total_capture_frames > 0) {
            print_recovery_progress_bar(std::min(scanned_frames, static_cast<size_t>(total_capture_frames)),
                                        static_cast<size_t>(total_capture_frames));
            std::cout << "\n";
        }
        std::cout << "recovered by window scan: " << recovered_count << "\n";
        return;
    }

    for (const uint32_t missing_index : missing_indices) {
        ++processed_missing;
        if (frames_buffer.find(missing_index) != frames_buffer.end()) {
            if (processed_missing == missing_indices.size() ||
                processed_missing == 1 ||
                processed_missing - last_reported >= 10) {
                print_recovery_progress_bar(processed_missing, missing_indices.size());
                last_reported = processed_missing;
            }
            continue;
        }

        const int approx_start = static_cast<int>(std::floor((kLeadInFrames + missing_index) * capture_ratio));
        const int approx_end = static_cast<int>(std::ceil((kLeadInFrames + missing_index + 1) * capture_ratio));
        int start_frame = std::max(0, approx_start - kWindowMarginFrames);
        int end_frame = approx_end + kWindowMarginFrames;
        if (total_capture_frames > 0) {
            end_frame = std::min(end_frame, total_capture_frames - 1);
        }
        if (end_frame < start_frame) {
            continue;
        }

        if (!cap.set(cv::CAP_PROP_POS_FRAMES, start_frame)) {
            continue;
        }

        for (int capture_index = start_frame; capture_index <= end_frame; ++capture_index) {
            if (!cap.read(frame) || frame.empty()) {
                break;
            }

            EncoderConfig candidate_cfg = cfg;
            std::vector<uint8_t> sample;
            if (!sample_frame(frame, sample, candidate_cfg)) {
                continue;
            }

            FrameHeader hdr{};
            std::vector<uint8_t> payload;
            if (!try_parse_data_frame(sample, candidate_cfg, hdr, payload)) {
                continue;
            }
            if (hdr.frame_index == missing_index) {
                frames_buffer[missing_index] = std::move(payload);
                ++recovered_count;
                break;
            }
        }

        if (processed_missing == missing_indices.size() ||
            processed_missing == 1 ||
            processed_missing - last_reported >= 10) {
            print_recovery_progress_bar(processed_missing, missing_indices.size());
            last_reported = processed_missing;
        }
    }
    print_recovery_progress_bar(processed_missing, missing_indices.size());
    std::cout << "\n"
              << "recovered by window scan: " << recovered_count << "\n";
}

} // namespace

int main(int argc, char* argv[]) {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    if (argc != 4 && argc != 5) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string video_path = argv[1];
    const std::string output_path = argv[2];
    const std::string validity_path = argv[3];
    const std::string reference_path = (argc == 5) ? argv[4] : "";

    if (!file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }
    if (!reference_path.empty() && !file_exists(reference_path)) {
        std::cerr << "Error: reference file not found: " << reference_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    EncoderConfig cfg = make_default_encoder_config();

    std::unordered_map<uint32_t, std::vector<uint8_t>> frames_buffer;
    uint32_t expected_total_frames = 0;
    uint64_t expected_total_bytes = 0;
    bool have_bootstrap = false;
    bool have_stream_header = false;
    if (!decode_frames_from_video(
            video_path,
            cfg,
            have_bootstrap,
            have_stream_header,
            expected_total_frames,
            expected_total_bytes,
            frames_buffer)) {
        std::cerr << "Error: failed while reading the video.\n";
        return static_cast<int>(ExitCode::DecodingError);
    }

    if (expected_total_frames == 0 && !frames_buffer.empty()) {
        uint32_t max_index = 0;
        for (const auto& [frame_index, _] : frames_buffer) {
            (void)_;
            max_index = std::max(max_index, frame_index);
        }
        expected_total_frames = max_index + 1;
    }

    if (expected_total_frames > 0 && frames_buffer.size() < expected_total_frames) {
        recover_missing_frames_from_video_windows(
            video_path,
            cfg,
            expected_total_frames,
            frames_buffer);
    }

    std::vector<uint8_t> recovered;
    std::vector<uint8_t> frame_validity_mask;
    uint64_t recovered_bytes = 0;
    if (expected_total_bytes > 0) {
        frame_validity_mask.reserve(static_cast<size_t>(expected_total_bytes));
    }
    std::cout << "decoded frames : " << frames_buffer.size() << "\n";
    std::cout << "total frames   : " << expected_total_frames << "\n";
    if (expected_total_frames > 0) {
        if (expected_total_bytes == 0) {
            expected_total_bytes = infer_total_bytes_from_frames(frames_buffer, expected_total_frames, cfg);
        }

        uint64_t current_size = 0;
        for (uint32_t i = 0; i < expected_total_frames; ++i) {
            const auto it = frames_buffer.find(i);
            if (it != frames_buffer.end() && !it->second.empty()) {
                recovered.insert(recovered.end(), it->second.begin(), it->second.end());
                frame_validity_mask.insert(frame_validity_mask.end(), it->second.size(), 0xFF);
                current_size += it->second.size();
                recovered_bytes += it->second.size();
                continue;
            }

            size_t pad_size = static_cast<size_t>(cfg.payload_bytes_per_frame);
            if (expected_total_bytes > 0 && i == expected_total_frames - 1) {
                const uint64_t remaining =
                    expected_total_bytes > current_size ? expected_total_bytes - current_size : 0;
                pad_size = static_cast<size_t>(std::min<uint64_t>(pad_size, remaining));
            } else if (expected_total_bytes > 0 && current_size + pad_size > expected_total_bytes) {
                pad_size = static_cast<size_t>(expected_total_bytes - current_size);
            }
            recovered.insert(recovered.end(), pad_size, 0x00);
            frame_validity_mask.insert(frame_validity_mask.end(), pad_size, 0x00);
            current_size += pad_size;
        }

        if (expected_total_bytes > 0 && recovered.size() < expected_total_bytes) {
            const size_t tail_pad_size =
                static_cast<size_t>(expected_total_bytes - recovered.size());
            recovered.insert(recovered.end(), tail_pad_size, 0x00);
            frame_validity_mask.insert(frame_validity_mask.end(), tail_pad_size, 0x00);
        }

        if (expected_total_bytes > 0 && recovered.size() > expected_total_bytes) {
            recovered.resize(static_cast<size_t>(expected_total_bytes));
            frame_validity_mask.resize(static_cast<size_t>(expected_total_bytes));
        }
    }

    try {
        write_binary_file(output_path, recovered);
        if (!reference_path.empty()) {
            const auto reference = read_binary_file(reference_path);
            std::vector<uint8_t> validity_exact(reference.size(), 0x00);
            const size_t compare_len = std::min(reference.size(), recovered.size());
            size_t matched = 0;
            size_t compared = 0;
            for (size_t i = 0; i < compare_len; ++i) {
                if (i >= frame_validity_mask.size() || frame_validity_mask[i] == 0x00) {
                    continue;
                }
                ++compared;
                if (reference[i] == recovered[i]) {
                    validity_exact[i] = 0xFF;
                    ++matched;
                }
            }

            write_binary_file(validity_path, validity_exact);
            const size_t abstained = compare_len > compared ? compare_len - compared : 0;
            std::cout << "output bytes   : " << recovered.size() << "\n"
                      << "reference bytes: " << reference.size() << "\n"
                      << "validity bytes : " << validity_exact.size() << "\n"
                      << "compared bytes : " << compared << "\n"
                      << "recovered bytes: " << recovered_bytes << "\n"
                      << "abstained bytes: " << abstained << "\n"
                      << "matched bytes  : " << matched << "\n"
                      << "accuracy       : "
                      << std::fixed << std::setprecision(2)
                      << (compared == 0
                              ? 0.0
                              : 100.0 * static_cast<double>(matched) /
                                    static_cast<double>(compared))
                      << "%\n"
                      << "full accuracy  : "
                      << std::fixed << std::setprecision(2)
                      << (reference.empty()
                              ? 0.0
                              : 100.0 * static_cast<double>(matched) /
                                    static_cast<double>(reference.size()))
                      << "%\n";
        } else {
            write_binary_file(validity_path, frame_validity_mask);
            std::cout << "output bytes   : " << recovered.size() << "\n"
                      << "recovered bytes: " << recovered_bytes << "\n"
                      << "abstained bytes: "
                      << (recovered.size() > static_cast<size_t>(recovered_bytes)
                              ? recovered.size() - static_cast<size_t>(recovered_bytes)
                              : 0)
                      << "\n"
                      << "validity bytes : " << frame_validity_mask.size() << "\n";
        }
    } catch (const std::exception& ex) {
        std::cerr << "Error writing output: " << ex.what() << "\n";
        return static_cast<int>(ExitCode::IoError);
    }
    return static_cast<int>(ExitCode::Ok);
}
