#include "common.hpp"
#include "io.hpp"
#include "codec.hpp"
#include "rs.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>
#include <unordered_map>
#include <filesystem>
#include <iomanip>
#include <array>
#include <clocale>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

using namespace camcom;
namespace fs = std::filesystem;

static void init_console_utf8() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::setlocale(LC_ALL, ".UTF-8");
}

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.mp4> <output.bin> [reference_input.bin]\n"
        << "\n"
        << "  input.mp4          Path to the video file to decode.\n"
        << "  output.bin         Path where the decoded binary data will be written.\n"
        << "  reference_input.bin(Optional) Path to original input.bin for accuracy report.\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " input.mp4 output.bin input.bin\n";
}

static uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
        (static_cast<uint32_t>(p[1]) << 8) |
        (static_cast<uint32_t>(p[2]) << 16) |
        (static_cast<uint32_t>(p[3]) << 24);
}

static uint64_t read_u64_le(const uint8_t* p) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(p[i]) << (8 * i);
    }
    return value;
}

static bool is_black_frame(const cv::Mat& frame) {
    cv::Scalar mean = cv::mean(frame);
    return mean[0] < 5 && mean[1] < 5 && mean[2] < 5;
}

static bool decode_rs(std::vector<uint8_t>& codeword, uint32_t nsym) {
    if (nsym == 0) {
        return true;
    }
    // 当前 RS 实现基于 GF(256) 仅稳定支持 n<=255 的码长。
    // 数据帧码字远大于 255 时，跳过 RS 纠错，后续依赖帧头 + CRC32 做有效性校验。
    if (codeword.size() > 255) {
        return true;
    }
    return rs::decode(codeword, static_cast<int>(nsym));
}

static bool is_sane_config_fields(uint32_t rs_nsym,
    uint32_t cell_size,
    uint32_t cells_per_row,
    uint32_t payload_per,
    uint32_t fps,
    uint32_t reference_block_size) {
    if (rs_nsym > 255) return false;
    if (cell_size < 2 || cell_size > 64) return false;
    if (cells_per_row < 8 || cells_per_row > 1024) return false;
    if (payload_per == 0 || payload_per > (1u << 20)) return false;
    if (fps == 0 || fps > 120) return false;
    if (reference_block_size == 0 || reference_block_size > 16) return false;
    return true;
}

static bool try_parse_bootstrap(const std::vector<uint8_t>& sample, EncoderConfig& cfg, uint32_t& stream_rs_nsym) {
    // 启动帧：用于在正式解码前恢复基础参数（网格、纠错等）。
    constexpr size_t kBootstrapBytes = 4 + 1 + 6 * 4;
    if (sample.size() < kBootstrapBytes) {
        return false;
    }

    // bootstrap 应该位于采样起始区域附近，限制扫描窗口可显著降低误命中概率。
    const size_t max_scan_off = std::min(sample.size() - kBootstrapBytes, static_cast<size_t>(32));
    for (size_t off = 0; off <= max_scan_off; ++off) {
        const uint8_t* p = sample.data() + static_cast<std::ptrdiff_t>(off);
        uint32_t magic = read_u32_le(p); p += 4;
        uint8_t version = *p; p += 1;
        if (magic != MAGIC || version != FORMAT_VERSION) {
            continue;
        }

        uint32_t rs_nsym = read_u32_le(p); p += 4;
        uint32_t cell_size = read_u32_le(p); p += 4;
        uint32_t cells_per_row = read_u32_le(p); p += 4;
        uint32_t payload_per = read_u32_le(p); p += 4;
        uint32_t fps = read_u32_le(p); p += 4;
        uint32_t reference_block_size = read_u32_le(p); p += 4;

        if (!is_sane_config_fields(rs_nsym, cell_size, cells_per_row, payload_per, fps, reference_block_size)) {
            continue;
        }

        cfg.cell_size = static_cast<int>(cell_size);
        cfg.fps = static_cast<int>(fps);
        cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
        cfg.cells_per_row = static_cast<int>(cells_per_row);
        cfg.rs_nsym = static_cast<int>(rs_nsym);
        cfg.reference_block_size = static_cast<int>(reference_block_size);
        stream_rs_nsym = rs_nsym;
        return true;
    }

    return false;
}

static bool try_parse_stream_header(const std::vector<uint8_t>& sample, uint32_t& stream_rs_nsym, EncoderConfig& cfg, uint32_t& expected_total_frames, uint64_t& expected_total_bytes) {
    // 流头帧：携带总帧数、总字节数以及编码配置，后续按该配置解码数据帧。
    constexpr size_t kStreamHeaderBytes = 4 + 1 + 8 + 1 + 6 * 4;
    const size_t codeword_len = kStreamHeaderBytes + static_cast<size_t>(stream_rs_nsym);
    if (sample.size() < codeword_len) {
        return false;
    }


    for (size_t off = 0; off + codeword_len <= sample.size(); ++off) {
        std::vector<uint8_t> codeword(
            sample.begin() + static_cast<std::ptrdiff_t>(off),
            sample.begin() + static_cast<std::ptrdiff_t>(off + codeword_len));
        if (!decode_rs(codeword, stream_rs_nsym)) {
            continue;
        }

        const uint8_t* p = codeword.data();
        uint32_t magic = read_u32_le(p); p += 4;
        uint8_t version = *p; p += 1;
        if (magic != MAGIC || version != FORMAT_VERSION) {
            continue;
        }

        uint64_t total_data = read_u64_le(p); p += 8;
        uint8_t encoding = *p; p += 1;
        (void)encoding;
        uint32_t fps = read_u32_le(p); p += 4;
        uint32_t cell_size = read_u32_le(p); p += 4;
        uint32_t rs_nsym = read_u32_le(p); p += 4;
        uint32_t payload_per = read_u32_le(p); p += 4;
        uint32_t cells_per_row = read_u32_le(p); p += 4;
        uint32_t total_frames_hdr = read_u32_le(p); p += 4;

        if (total_frames_hdr == 0) {
            continue;
        }
        if (!is_sane_config_fields(rs_nsym, cell_size, cells_per_row, payload_per, fps, cfg.reference_block_size > 0 ? static_cast<uint32_t>(cfg.reference_block_size) : 2u)) {
            continue;
        }

        cfg.cell_size = static_cast<int>(cell_size);
        cfg.fps = static_cast<int>(fps);
        cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
        cfg.cells_per_row = static_cast<int>(cells_per_row);
        cfg.rs_nsym = static_cast<int>(rs_nsym);
        stream_rs_nsym = rs_nsym;
        expected_total_frames = total_frames_hdr;
        expected_total_bytes = total_data;
        return true;
    }

    return false;
}

static bool try_parse_data_frame(const std::vector<uint8_t>& sample, uint32_t stream_rs_nsym, const EncoderConfig& cfg, FrameHeader& hdr, std::vector<uint8_t>& payload_out) {
    // 数据帧解析策略：先按“满负载长度”尝试，再回退到“按帧头声明长度”尝试。
    constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
    if (sample.size() < kFrameHeaderBytes + static_cast<size_t>(stream_rs_nsym)) {
        return false;
    }

    const size_t full_payload_codeword = kFrameHeaderBytes + static_cast<size_t>(cfg.payload_bytes_per_frame) + static_cast<size_t>(stream_rs_nsym);
    if (cfg.payload_bytes_per_frame > 0 && sample.size() >= full_payload_codeword) {
        for (size_t off = 0; off + full_payload_codeword <= sample.size(); ++off) {
            std::vector<uint8_t> codeword(
                sample.begin() + static_cast<std::ptrdiff_t>(off),
                sample.begin() + static_cast<std::ptrdiff_t>(off + full_payload_codeword));
            if (!decode_rs(codeword, stream_rs_nsym)) {
                continue;
            }

            const uint8_t* p = codeword.data();
            FrameHeader local{};
            local.magic = read_u32_le(p); p += 4;
            local.version = *p; p += 1;
            local.frame_index = read_u32_le(p); p += 4;
            local.total_frames = read_u32_le(p); p += 4;
            local.payload_bytes = read_u32_le(p); p += 4;
            local.checksum = read_u32_le(p); p += 4;

            if (local.magic != MAGIC || local.version != FORMAT_VERSION) {
                continue;
            }
            if (local.total_frames == 0 || local.frame_index >= local.total_frames) {
                continue;
            }
            if (local.payload_bytes == 0 || local.payload_bytes > static_cast<uint32_t>(cfg.payload_bytes_per_frame)) {
                continue;
            }

            const size_t payload_len = static_cast<size_t>(local.payload_bytes);
            if (p + payload_len > codeword.data() + codeword.size()) {
                continue;
            }

            const uint32_t calc = crc32(p, payload_len);
            if (calc != local.checksum) {
                continue;
            }

            hdr = local;
            payload_out.assign(p, p + payload_len);
            return true;
        }
    }

    // 快速回退：先扫描原始帧头候选，再按匹配长度执行 RS 解码。
    const size_t min_codeword_len = kFrameHeaderBytes + static_cast<size_t>(stream_rs_nsym);
    for (size_t off = 0; off + min_codeword_len <= sample.size(); ++off) {
        const uint8_t* raw = sample.data() + static_cast<std::ptrdiff_t>(off);
        if (read_u32_le(raw) != MAGIC || raw[4] != FORMAT_VERSION) {
            continue;
        }

        const uint32_t payload_bytes = read_u32_le(raw + 13);
        if (payload_bytes == 0 || payload_bytes > static_cast<uint32_t>(cfg.payload_bytes_per_frame)) {
            continue;
        }

        const size_t codeword_len = kFrameHeaderBytes + static_cast<size_t>(payload_bytes) + static_cast<size_t>(stream_rs_nsym);
        if (off + codeword_len > sample.size()) {
            continue;
        }

        std::vector<uint8_t> codeword(
            sample.begin() + static_cast<std::ptrdiff_t>(off),
            sample.begin() + static_cast<std::ptrdiff_t>(off + codeword_len));
        if (!decode_rs(codeword, stream_rs_nsym)) {
            continue;
        }

        const uint8_t* p = codeword.data();
        FrameHeader local{};
        local.magic = read_u32_le(p); p += 4;
        local.version = *p; p += 1;
        local.frame_index = read_u32_le(p); p += 4;
        local.total_frames = read_u32_le(p); p += 4;
        local.payload_bytes = read_u32_le(p); p += 4;
        local.checksum = read_u32_le(p); p += 4;

        if (local.magic != MAGIC || local.version != FORMAT_VERSION) {
            continue;
        }
        if (local.total_frames == 0 || local.frame_index >= local.total_frames) {
            continue;
        }

        const size_t payload_len = static_cast<size_t>(local.payload_bytes);
        if (p + payload_len > codeword.data() + codeword.size()) {
            continue;
        }

        const uint32_t calc = crc32(p, payload_len);
        if (calc != local.checksum) {
            continue;
        }

        hdr = local;
        payload_out.assign(p, p + payload_len);
        return true;
    }

    return false;
}

int main(int argc, char* argv[]) {
    init_console_utf8();
    std::cout << "[decoder] Starting...\n";
    std::cout << "[decoder] argc: " << argc << "\n";
    for (int i = 0; i < argc; ++i) {
        std::cout << "[decoder] argv[" << i << "]: " << argv[i] << "\n";
    }

    if (argc < 3 || argc > 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string video_path = argv[1];
    const std::string output_path = argv[2];
    const std::string reference_path = (argc == 4) ? argv[3] : "";

    std::cout << "[decoder] Input video: " << video_path << "\n";
    std::cout << "[decoder] Output file: " << output_path << "\n";
    if (!reference_path.empty()) {
        std::cout << "[decoder] Ref file   : " << reference_path << "\n";
    }

    if (!file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }
    std::cout << "[decoder] Video file exists\n";

    EncoderConfig cfg;
    // 启动默认参数；若检测到 bootstrap / stream header，会被真实参数覆盖。
    cfg.cell_size = 8;
    cfg.cells_per_row = 108;
    cfg.payload_bytes_per_frame = 2875;

    const std::string temp_dir = "temp_frames_dec";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    std::string ffmpeg_cmd = "ffmpeg -nostdin -y -i " + video_path + " " + temp_dir + "/frame_%d.png";
    // 先把视频拆帧，后续逐帧走采样 + 协议解析流程。
    std::cout << "[decoder] Running ffmpeg command: " << ffmpeg_cmd << "\n";
    int ffmpeg_result = system(ffmpeg_cmd.c_str());
    if (ffmpeg_result != 0) {
        std::cerr << "Error: ffmpeg command failed to extract frames.\n";
        fs::remove_all(temp_dir);
        return static_cast<int>(ExitCode::DecodingError);
    }

    std::unordered_map<uint32_t, std::vector<uint8_t>> frames_buffer;
    uint32_t expected_total_frames = 0;
    uint64_t expected_total_bytes = 0;
    bool have_bootstrap = false;
    bool have_stream_header = false;

    int sampled_frames = 0;
    int dedup_skipped_frames = 0;
    int sample_failed_frames = 0;
    int parsed_data_frames = 0;
    int bootstrap_hits = 0;
    int stream_header_hits = 0;
    int candidate_parse_attempts = 0;

    int orient_sample_hits = 0;
    int orient_data_hits = 0;

    uint32_t stream_rs_nsym = static_cast<uint32_t>(cfg.rs_nsym);
    cv::Mat prev_thumb_gray;

    int frame_count = 1;
    while (true) {
        // 逐帧处理：采样 -> 解析 bootstrap/stream header/data frame。
        std::string frame_file = temp_dir + "/frame_" + std::to_string(frame_count) + ".png";
        if (!fs::exists(frame_file)) {
            break;
        }

        cv::Mat frame = cv::imread(frame_file, cv::IMREAD_COLOR);
        if (frame.empty()) {
            frame_count++;
            continue;
        }

        std::cout << "[decoder] Processing frame " << frame_count << "\n";
        frame_count++;

        if (is_black_frame(frame)) {
            continue;
        }

        // 手机录屏/拍摄常出现大量重复帧，先做轻量去重，避免无效重算。
        {
            cv::Mat thumb, thumb_gray;
            cv::resize(frame, thumb, cv::Size(64, 64), 0, 0, cv::INTER_AREA);
            cv::cvtColor(thumb, thumb_gray, cv::COLOR_BGR2GRAY);
            if (!prev_thumb_gray.empty()) {
                cv::Mat diff;
                cv::absdiff(thumb_gray, prev_thumb_gray, diff);
                cv::Scalar mdiff = cv::mean(diff);
                if (mdiff[0] < 1.5) {
                    ++dedup_skipped_frames;
                    continue;
                }
            }
            prev_thumb_gray = thumb_gray;
        }

        // 4K 帧会显著拖慢检测；先等比缩放到可控尺寸。
        cv::Mat proc = frame;
        {
            const int max_side = std::max(frame.cols, frame.rows);
            constexpr int kDecodeMaxSide = 1280;
            if (max_side > kDecodeMaxSide) {
                const double scale = static_cast<double>(kDecodeMaxSide) / static_cast<double>(max_side);
                cv::resize(frame, proc, cv::Size(), scale, scale, cv::INTER_AREA);
            }
        }

        struct SampleCandidate {
            const char* scale_name;
            std::vector<uint8_t> bytes;
        };

        std::vector<SampleCandidate> sample_candidates;
        sample_candidates.reserve(4);

        auto collect_sample = [&](const char* scale_name, const cv::Mat& img) {
            std::vector<uint8_t> s;
            if (sample_frame(img, s, cfg) && !s.empty()) {
                sample_candidates.push_back({ scale_name, std::move(s) });
                ++orient_sample_hits;
            }
        };

        // 先尝试降采样版本；若尚未锁定流头，再补充一次原分辨率尝试以提升首帧参数识别率。
        collect_sample("scaled", proc);
        const bool need_bootstrap_probe = (!have_stream_header && sampled_frames < 160 && (proc.cols != frame.cols || proc.rows != frame.rows));
        if (need_bootstrap_probe) {
            collect_sample("full", frame);
        }

        ++sampled_frames;
        if (sample_candidates.empty()) {
            std::cout << "[decoder] Failed to sample frame\n";
            ++sample_failed_frames;
            continue;
        }

        if (sampled_frames <= 10 || sampled_frames % 120 == 0) {
            std::cout << "[decoder][diag] frame sampled ok, candidates=" << sample_candidates.size()
                << ", cfg(cell=" << cfg.cell_size
                << ", cols=" << cfg.cells_per_row
                << ", payload=" << cfg.payload_bytes_per_frame
                << ", rs=" << stream_rs_nsym << ")\n";
        }

        bool frame_decoded = false;
        for (auto& cand : sample_candidates) {
            ++candidate_parse_attempts;
            auto& sample = cand.bytes;
            if (!have_bootstrap && !have_stream_header) {
                // bootstrap 可能在压缩/采样噪声下不可读；尽力解析但不作为后续解码的硬门槛。
                if (try_parse_bootstrap(sample, cfg, stream_rs_nsym)) {
                    have_bootstrap = true;
                    ++bootstrap_hits;
                    std::cout << "[decoder] Found bootstrap frame with config: cell_size=" << cfg.cell_size << ", cells_per_row=" << cfg.cells_per_row << "\n";
                }
            }

            if (!have_stream_header) {
                // 允许在 bootstrap 缺失时直接尝试流头（优先使用默认/当前 RS 配置）。
                if (try_parse_stream_header(sample, stream_rs_nsym, cfg, expected_total_frames, expected_total_bytes)) {
                    have_stream_header = true;
                    ++stream_header_hits;
                    std::cout << "[decoder] Found stream header: total_frames=" << expected_total_frames << ", total_bytes=" << expected_total_bytes << "\n";
                }
            }

            FrameHeader hdr{};
            std::vector<uint8_t> payload;
            if (!try_parse_data_frame(sample, stream_rs_nsym, cfg, hdr, payload)) {
                continue;
            }

            if (hdr.total_frames > 0 && expected_total_frames == 0) {
                expected_total_frames = hdr.total_frames;
            }

            auto it = frames_buffer.find(hdr.frame_index);
            // 同一 frame_index 只保留第一份有效载荷，避免重复帧覆盖。
            if (it == frames_buffer.end() || it->second.empty()) {
                frames_buffer[hdr.frame_index] = std::move(payload);
                std::cout << "[decoder] Decoded frame " << hdr.frame_index << " of " << hdr.total_frames << "\n";
                ++parsed_data_frames;
                ++orient_data_hits;
            }

            frame_decoded = true;
            break;
        }

        if (!frame_decoded) {
            if (sampled_frames <= 10 || sampled_frames % 120 == 0) {
                auto count_magic_hits = [&](const std::vector<uint8_t>& s) {
                    int hits = 0;
                    if (s.size() < 4) return 0;
                    for (size_t i = 0; i + 4 <= s.size(); ++i) {
                        if (read_u32_le(s.data() + static_cast<std::ptrdiff_t>(i)) == MAGIC) {
                            ++hits;
                        }
                    }
                    return hits;
                };

                std::cout << "[decoder][diag] frame parse miss: candidates=" << sample_candidates.size()
                    << " [";
                for (size_t i = 0; i < sample_candidates.size(); ++i) {
                    const auto& c = sample_candidates[i];
                    std::cout << c.scale_name << ":len=" << c.bytes.size()
                        << ",magic=" << count_magic_hits(c.bytes);
                    if (i + 1 < sample_candidates.size()) std::cout << ", ";
                }
                std::cout << "]\n";
            }
            continue;
        }
    }

    // 重组输出数据
    std::vector<uint8_t> recovered;

    if (expected_total_frames == 0) {
        // 若未拿到流头，则根据已解出的最大帧号估算总帧数。
        if (!frames_buffer.empty()) {
            uint32_t max_index = 0;
            for (const auto& kv : frames_buffer) {
                max_index = std::max(max_index, kv.first);
            }
            expected_total_frames = max_index + 1;
        }
    }

    if (expected_total_frames == 0) {
        std::cerr << "Warning: no frames decoded successfully." << std::endl;
    }
    else {
        uint64_t current_size = 0;
        for (uint32_t i = 0; i < expected_total_frames; ++i) {
            auto it = frames_buffer.find(i);
            size_t frame_payload_size = 0;

            if (it != frames_buffer.end() && !it->second.empty()) {
                frame_payload_size = it->second.size();
                recovered.insert(recovered.end(), it->second.begin(), it->second.end());
                current_size += frame_payload_size;
            }
            else {
                // 缺失帧：使用 0 填充
                std::cerr << "Warning: missing frame " << i << "\n";
                // 估算缺失帧应填充的字节数
                size_t pad_size = static_cast<size_t>(cfg.payload_bytes_per_frame);
                if (expected_total_bytes > 0 && i == expected_total_frames - 1) {
                    uint64_t remaining = (expected_total_bytes > current_size) ? expected_total_bytes - current_size : 0;
                    pad_size = static_cast<size_t>(std::min(static_cast<uint64_t>(pad_size), remaining));
                } else if (expected_total_bytes > 0 && current_size + pad_size > expected_total_bytes) {
                    pad_size = static_cast<size_t>(expected_total_bytes - current_size);
                }

                recovered.insert(recovered.end(), pad_size, 0x00);
                current_size += pad_size;
            }
        }

        if (expected_total_bytes > 0 && recovered.size() > expected_total_bytes) {
            recovered.resize(static_cast<size_t>(expected_total_bytes));
        }
    }

    // 写出解码结果
    try {
        std::cout << "[decoder] Writing output file: " << output_path << "\n";
        write_binary_file(output_path, recovered);
        std::cout << "[decoder] Output file written successfully\n";

        if (!reference_path.empty()) {
            // 可选：与原始输入做逐字节比对，输出准确率与掩码文件。
            const auto reference = read_binary_file(reference_path);
            const size_t compare_len = std::min(reference.size(), recovered.size());
            size_t matched = 0;
            std::vector<uint8_t> v1_mask(reference.size(), 0x00);
            for (size_t i = 0; i < compare_len; ++i) {
                if (reference[i] == recovered[i]) {
                    ++matched;
                    v1_mask[i] = 0xFF;
                }
            }

            const double accuracy = reference.empty()
                ? 0.0
                : (100.0 * static_cast<double>(matched) / static_cast<double>(reference.size()));

            write_binary_file("v1.bin", v1_mask);

            std::cout << "[decoder] Compare report:\n"
                << "  input bytes    : " << reference.size() << "\n"
                << "  output bytes   : " << recovered.size() << "\n"
                << "  compared bytes : " << compare_len << "\n"
                << "  matched bytes  : " << matched << "\n"
                << "  accuracy       : " << std::fixed << std::setprecision(2) << accuracy << "%\n"
                << "  v1 mask file   : v1.bin (0xFF=correct, 0x00=wrong)\n";
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Error writing output: " << ex.what() << "\n";
        fs::remove_all(temp_dir);
        return static_cast<int>(ExitCode::IoError);
    }

    fs::remove_all(temp_dir);
    std::cout << "[decoder] Stats: sampled=" << sampled_frames
        << ", dedup_skipped=" << dedup_skipped_frames
        << ", sample_failed=" << sample_failed_frames
        << ", parsed_data=" << parsed_data_frames << "\n";
    std::cout << "[decoder] ParseStats: bootstrap_hits=" << bootstrap_hits
        << ", stream_header_hits=" << stream_header_hits
        << ", candidate_parse_attempts=" << candidate_parse_attempts
        << ", buffered_frames=" << frames_buffer.size() << "\n";
    std::cout << "[decoder] OrientStats(sample_hits): single=" << orient_sample_hits << "\n";
    std::cout << "[decoder] OrientStats(data_hits): single=" << orient_data_hits << "\n";
    std::cout << "[decoder] Done. Recovered bytes=" << recovered.size() << "\n";
    return static_cast<int>(ExitCode::Ok);
}