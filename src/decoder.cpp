// Decoder entry point for extracting frames and rebuilding binary data.
#include "codec.hpp"
#include "common.hpp"
#include "ffmpeg.hpp"
#include "io.hpp"
#include "rs.hpp"

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using namespace camcom;
namespace fs = std::filesystem;

namespace {

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.mp4> <output.bin> [reference_input.bin]\n"
        << "Example: " << argv0 << " input.mp4 output.bin input.bin\n";
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

bool is_black_frame(const cv::Mat& frame) {
    const cv::Scalar mean = cv::mean(frame);
    return mean[0] < 5 && mean[1] < 5 && mean[2] < 5;
}

bool decode_rs(std::vector<uint8_t>& codeword, uint32_t nsym) {
    if (nsym == 0 || codeword.size() > 255) {
        return true;
    }
    return rs::decode(codeword, static_cast<int>(nsym));
}

bool try_parse_bootstrap(const std::vector<uint8_t>& sample, EncoderConfig& cfg, uint32_t& stream_rs_nsym) {
    constexpr size_t kBootstrapBytes = 4 + 1 + 6 * 4;
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

        const uint32_t rs_nsym = read_u32_le(p); p += 4;
        const uint32_t cell_size = read_u32_le(p); p += 4;
        const uint32_t cells_per_row = read_u32_le(p); p += 4;
        const uint32_t payload_per = read_u32_le(p); p += 4;
        const uint32_t fps = read_u32_le(p); p += 4;
        const uint32_t reference_block_size = read_u32_le(p); p += 4;

        const bool sane =
            rs_nsym <= 255 &&
            cell_size >= 8 && cell_size <= 64 &&
            cells_per_row >= 16 && cells_per_row <= 256 &&
            payload_per >= 64 && payload_per <= 4096 &&
            fps >= 1 && fps <= 30 &&
            reference_block_size >= 1 && reference_block_size <= 8;
        if (!sane) {
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

bool try_parse_stream_header(
    const std::vector<uint8_t>& sample,
    uint32_t& stream_rs_nsym,
    EncoderConfig& cfg,
    uint32_t& expected_total_frames,
    uint64_t& expected_total_bytes) {
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
        const uint32_t magic = read_u32_le(p); p += 4;
        const uint8_t version = *p; p += 1;
        if (magic != MAGIC || version != FORMAT_VERSION) {
            continue;
        }

        const uint64_t total_data = read_u64_le(p); p += 8;
        p += 1; // encoding
        const uint32_t fps = read_u32_le(p); p += 4;
        const uint32_t cell_size = read_u32_le(p); p += 4;
        const uint32_t rs_nsym = read_u32_le(p); p += 4;
        const uint32_t payload_per = read_u32_le(p); p += 4;
        const uint32_t cells_per_row = read_u32_le(p); p += 4;
        const uint32_t total_frames_hdr = read_u32_le(p); p += 4;

        const bool sane =
            rs_nsym <= 255 &&
            cell_size >= 8 && cell_size <= 64 &&
            payload_per >= 64 && payload_per <= 4096 &&
            cells_per_row >= 16 && cells_per_row <= 256 &&
            total_frames_hdr >= 1 && total_frames_hdr <= 100000 &&
            fps >= 1 && fps <= 30 &&
            total_data > 0;
        if (!sane) {
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

bool try_parse_data_frame(
    const std::vector<uint8_t>& sample,
    uint32_t stream_rs_nsym,
    const EncoderConfig& cfg,
    FrameHeader& hdr,
    std::vector<uint8_t>& payload_out) {
    constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
    if (sample.size() < kFrameHeaderBytes + static_cast<size_t>(stream_rs_nsym)) {
        return false;
    }

    const size_t full_codeword_len =
        kFrameHeaderBytes + static_cast<size_t>(cfg.payload_bytes_per_frame) + static_cast<size_t>(stream_rs_nsym);
    if (cfg.payload_bytes_per_frame > 0 && sample.size() >= full_codeword_len) {
        for (size_t off = 0; off + full_codeword_len <= sample.size(); ++off) {
            std::vector<uint8_t> codeword(
                sample.begin() + static_cast<std::ptrdiff_t>(off),
                sample.begin() + static_cast<std::ptrdiff_t>(off + full_codeword_len));
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
            if (crc32(p, payload_len) != local.checksum) {
                continue;
            }

            hdr = local;
            payload_out.assign(p, p + payload_len);
            return true;
        }
    }

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

        const size_t codeword_len =
            kFrameHeaderBytes + static_cast<size_t>(payload_bytes) + static_cast<size_t>(stream_rs_nsym);
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
        if (crc32(p, payload_len) != local.checksum) {
            continue;
        }

        hdr = local;
        payload_out.assign(p, p + payload_len);
        return true;
    }

    return false;
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
        static_cast<uint64_t>(cfg.payload_bytes_per_frame) +
        static_cast<uint64_t>(last_it->second.size());
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string video_path = argv[1];
    const std::string output_path = argv[2];
    const std::string reference_path = (argc == 4) ? argv[3] : "";

    if (!file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    EncoderConfig cfg;
    cfg.cell_size = 16;
    cfg.cells_per_row = 108;
    cfg.payload_bytes_per_frame = 2875;

    const std::string temp_dir = "temp_frames_dec";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    const std::string ffmpeg = find_ffmpeg_executable(argv[0]);
    const int ffmpeg_result = run_process({
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-nostdin",
        "-y",
        "-i",
        video_path,
        "-vf",
        "fps=15",
        temp_dir + "/frame_%d.png",
    });
    if (ffmpeg_result != 0) {
        fs::remove_all(temp_dir);
        std::cerr << "Error: ffmpeg failed while extracting frames.\n";
        return static_cast<int>(ExitCode::DecodingError);
    }

    std::unordered_map<uint32_t, std::vector<uint8_t>> frames_buffer;
    uint32_t expected_total_frames = 0;
    uint64_t expected_total_bytes = 0;
    bool have_bootstrap = false;
    bool have_stream_header = false;
    uint32_t stream_rs_nsym = static_cast<uint32_t>(cfg.rs_nsym);

    for (int frame_count = 1;; ++frame_count) {
        const std::string frame_file = temp_dir + "/frame_" + std::to_string(frame_count) + ".png";
        if (!fs::exists(frame_file)) {
            break;
        }

        cv::Mat frame = cv::imread(frame_file, cv::IMREAD_COLOR);
        if (frame.empty() || is_black_frame(frame)) {
            continue;
        }

        std::vector<uint8_t> sample;
        if (!sample_frame(frame, sample, cfg)) {
            continue;
        }

        if (!have_bootstrap) {
            have_bootstrap = try_parse_bootstrap(sample, cfg, stream_rs_nsym);
        }
        if (!have_stream_header) {
            have_stream_header = try_parse_stream_header(
                sample, stream_rs_nsym, cfg, expected_total_frames, expected_total_bytes);
        }

        FrameHeader hdr{};
        std::vector<uint8_t> payload;
        if (!try_parse_data_frame(sample, stream_rs_nsym, cfg, hdr, payload)) {
            continue;
        }

        if (expected_total_frames == 0 && hdr.total_frames > 0) {
            expected_total_frames = hdr.total_frames;
        }

        auto [it, inserted] = frames_buffer.emplace(hdr.frame_index, std::move(payload));
        if (!inserted && it->second.empty()) {
            it->second = std::move(payload);
        }
    }

    if (expected_total_frames == 0 && !frames_buffer.empty()) {
        uint32_t max_index = 0;
        for (const auto& [frame_index, _] : frames_buffer) {
            (void)_;
            max_index = std::max(max_index, frame_index);
        }
        expected_total_frames = max_index + 1;
    }

    std::vector<uint8_t> recovered;
    if (expected_total_frames > 0) {
        if (expected_total_bytes == 0) {
            expected_total_bytes = infer_total_bytes_from_frames(frames_buffer, expected_total_frames, cfg);
        }

        uint64_t current_size = 0;
        for (uint32_t i = 0; i < expected_total_frames; ++i) {
            const auto it = frames_buffer.find(i);
            if (it != frames_buffer.end() && !it->second.empty()) {
                recovered.insert(recovered.end(), it->second.begin(), it->second.end());
                current_size += it->second.size();
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
            current_size += pad_size;
        }

        if (expected_total_bytes > 0 && recovered.size() > expected_total_bytes) {
            recovered.resize(static_cast<size_t>(expected_total_bytes));
        }
    }

    try {
        write_binary_file(output_path, recovered);

        if (!reference_path.empty()) {
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
                : 100.0 * static_cast<double>(matched) / static_cast<double>(reference.size());
            write_binary_file("v1.bin", v1_mask);

            std::cout << "input bytes    : " << reference.size() << "\n"
                << "output bytes   : " << recovered.size() << "\n"
                << "compared bytes : " << compare_len << "\n"
                << "matched bytes  : " << matched << "\n"
                << "accuracy       : " << std::fixed << std::setprecision(2) << accuracy << "%\n";
        }
    } catch (const std::exception& ex) {
        fs::remove_all(temp_dir);
        std::cerr << "Error writing output: " << ex.what() << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    fs::remove_all(temp_dir);
    return static_cast<int>(ExitCode::Ok);
}
