// 实现 encode 主程序，将二进制输入编码为可视化帧并生成视频。
#include "codec.hpp"
#include "common.hpp"
#include "fec.hpp"
#include "ffmpeg.hpp"
#include "io.hpp"

#include <clocale>
#include <cmath>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

using namespace camcom;
namespace fs = std::filesystem;

namespace {

constexpr int kFixedOutputFps = 20;
constexpr int kBootstrapRepeat = 0;
constexpr int kStreamHeaderRepeat = 0;
constexpr int kLastFrameRepeat = 0;
constexpr int kFinalHeaderRepeat = 0;
constexpr int kFixedOverheadFrames = kBootstrapRepeat + kStreamHeaderRepeat + kFinalHeaderRepeat;
constexpr size_t kFrameHeaderBytes = 4 + 1 + 4 + 4 + 4 + 4;
constexpr size_t kPaddingFramePayloadBytes = 1;

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <duration_milliseconds>\n"
        << "Example: " << argv0 << " payload.bin out.mp4 1000\n";
}

void init_console_utf8() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::setlocale(LC_ALL, ".UTF-8");
}

void serialize_u8(std::vector<uint8_t>& buf, uint8_t v) { buf.push_back(v); }

void serialize_u32(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}

void serialize_u64(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
    }
}

std::vector<uint8_t> build_bootstrap(const EncoderConfig& cfg) {
    std::vector<uint8_t> buf;
    serialize_u32(buf, MAGIC);
    serialize_u8(buf, FORMAT_VERSION);
    serialize_u32(buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(buf, static_cast<uint32_t>(cfg.reference_block_size));
    return buf;
}

std::vector<uint8_t> build_stream_header(const EncoderConfig& cfg, size_t total_bytes, uint32_t total_frames) {
    std::vector<uint8_t> buf;
    serialize_u32(buf, MAGIC);
    serialize_u8(buf, FORMAT_VERSION);
    serialize_u64(buf, static_cast<uint64_t>(total_bytes));
    serialize_u8(buf, static_cast<uint8_t>(Encoding::Binary));
    serialize_u32(buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(buf, total_frames);
    return buf;
}

std::vector<uint8_t> build_frame_codeword(
    uint32_t frame_index,
    uint32_t total_frames,
    const EncoderConfig& cfg,
    const std::vector<uint8_t>& source_payload) {
    std::vector<uint8_t> frame_buf;
    serialize_u32(frame_buf, MAGIC);
    serialize_u8(frame_buf, FORMAT_VERSION);
    serialize_u32(frame_buf, frame_index);
    serialize_u32(frame_buf, total_frames);
    serialize_u32(frame_buf, static_cast<uint32_t>(source_payload.size()));

    const size_t checksum_pos = frame_buf.size();
    serialize_u32(frame_buf, 0);
    const std::vector<uint8_t> encoded_payload =
        fec_encode_payload(source_payload, static_cast<size_t>(cfg.payload_bytes_per_frame));
    frame_buf.insert(frame_buf.end(), encoded_payload.begin(), encoded_payload.end());

    const uint32_t checksum = crc32(source_payload.data(), source_payload.size());
    frame_buf[checksum_pos + 0] = static_cast<uint8_t>(checksum & 0xFFu);
    frame_buf[checksum_pos + 1] = static_cast<uint8_t>((checksum >> 8) & 0xFFu);
    frame_buf[checksum_pos + 2] = static_cast<uint8_t>((checksum >> 16) & 0xFFu);
    frame_buf[checksum_pos + 3] = static_cast<uint8_t>((checksum >> 24) & 0xFFu);

    return frame_buf;
}

std::vector<uint8_t> slice_payload(
    const std::vector<uint8_t>& data,
    size_t offset,
    size_t chunk) {
    return std::vector<uint8_t>(
        data.begin() + static_cast<std::ptrdiff_t>(offset),
        data.begin() + static_cast<std::ptrdiff_t>(offset + chunk));
}

void write_frame(const cv::Mat& img, const std::string& temp_dir, size_t& frame_count) {
    cv::imwrite(temp_dir + "/frame_" + std::to_string(frame_count++) + ".png", img);
}

void append_repeated_frame(
    const std::vector<uint8_t>& payload,
    const EncoderConfig& cfg,
    int render_rows,
    const std::string& temp_dir,
    size_t& frame_count,
    int repeat) {
    if (repeat <= 0 || payload.empty()) {
        return;
    }

    cv::Mat frame_img;
    render_frame(frame_img, payload, cfg, render_rows);
    for (int i = 0; i < repeat; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }
}

size_t calculate_total_output_frames(uint32_t data_frames) {
    size_t total =
        static_cast<size_t>(kFixedOverheadFrames) +
        static_cast<size_t>(data_frames);
    if (data_frames > 0) {
        total += static_cast<size_t>(kLastFrameRepeat);
    }
    return total;
}

size_t max_chunk_bytes_for_frame_budget(
    const EncoderConfig& cfg,
    size_t prefix_bytes,
    size_t chunk_upper_bound) {
    const size_t frame_byte_budget =
        kFrameHeaderBytes + static_cast<size_t>(cfg.payload_bytes_per_frame);
    if (prefix_bytes + kFrameHeaderBytes >= frame_byte_budget) {
        return 0;
    }

    const size_t max_source_bytes =
        fec_max_source_size_for_encoded_capacity(
            static_cast<size_t>(cfg.payload_bytes_per_frame));
    size_t hi = std::min(chunk_upper_bound, max_source_bytes);
    if (hi == 0) {
        return 0;
    }

    size_t lo = 1;
    size_t best = 0;
    while (lo <= hi) {
        const size_t mid = lo + (hi - lo) / 2;
        const size_t encoded_payload = fec_encoded_size_for_source_size(mid);
        const size_t total_bytes = prefix_bytes + kFrameHeaderBytes + encoded_payload;
        if (total_bytes <= frame_byte_budget) {
            best = mid;
            lo = mid + 1;
        } else {
            if (mid == 0) {
                break;
            }
            hi = mid - 1;
        }
    }
    return best;
}

} // namespace

int main(int argc, char* argv[]) {
    init_console_utf8();
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);

    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    double max_milliseconds = 0.0;
    try {
        max_milliseconds = std::stod(argv[3]);
    } catch (const std::exception&) {
        std::cerr << "Error: max_milliseconds must be a number.\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    if (!(max_milliseconds > 0.0)) {
        std::cerr << "Error: max_milliseconds must be > 0.\n";
        return static_cast<int>(ExitCode::BadArgs);
    }
    if (!file_exists(input_path)) {
        std::cerr << "Error: input file not found: " << input_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    const auto data = read_binary_file(input_path);

    EncoderConfig cfg = make_default_encoder_config(kFixedOutputFps);
    const FecConfig& fec_cfg = default_fec_config();

    const size_t payload_per_frame =
        fec_max_source_size_for_encoded_capacity(
            static_cast<size_t>(cfg.payload_bytes_per_frame),
            fec_cfg);
    const uint32_t full_data_frames =
        static_cast<uint32_t>((data.size() + payload_per_frame - 1) / payload_per_frame);

    const std::vector<uint8_t> bootstrap_buf = build_bootstrap(cfg);
    const std::vector<uint8_t> stream_buf_template = build_stream_header(cfg, 0, 0);
    const size_t first_frame_prefix_bytes = bootstrap_buf.size() + stream_buf_template.size();
    const size_t target_output_frames = static_cast<size_t>(std::llround(
        max_milliseconds * static_cast<double>(kFixedOutputFps) / 1000.0));
    if (target_output_frames == 0) {
        std::cerr << "Error: max_milliseconds is too small for fixed fps "
                  << kFixedOutputFps << ".\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    const size_t min_frames_for_data =
        static_cast<size_t>(kFixedOverheadFrames + kLastFrameRepeat + 1);
    if (target_output_frames < min_frames_for_data) {
        std::cerr << "Error: max_milliseconds is too small to carry payload data.\n"
                  << "  target frames    : " << target_output_frames << "\n"
                  << "  minimum required : " << min_frames_for_data << "\n"
                  << "  fixed fps        : " << kFixedOutputFps << "\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    const size_t max_data_frames_by_time =
        target_output_frames - static_cast<size_t>(kFixedOverheadFrames + kLastFrameRepeat);
    std::vector<size_t> frame_chunks;
    frame_chunks.reserve(max_data_frames_by_time);
    size_t encoded_total_bytes = 0;
    for (size_t i = 0; i < max_data_frames_by_time && encoded_total_bytes < data.size(); ++i) {
        const size_t prefix_bytes = (i == 0) ? first_frame_prefix_bytes : 0;
        const size_t chunk_upper_bound = data.size() - encoded_total_bytes;
        const size_t chunk = max_chunk_bytes_for_frame_budget(cfg, prefix_bytes, chunk_upper_bound);
        if (chunk == 0) {
            break;
        }
        frame_chunks.push_back(chunk);
        encoded_total_bytes += chunk;
    }

    if (frame_chunks.empty() || encoded_total_bytes == 0) {
        std::cerr << "Error: no payload frame can be encoded under current max_milliseconds.\n";
        return static_cast<int>(ExitCode::BadArgs);
    }
    while (frame_chunks.size() < max_data_frames_by_time) {
        frame_chunks.push_back(kPaddingFramePayloadBytes);
    }
    const uint32_t encoded_data_frames = static_cast<uint32_t>(frame_chunks.size());
    const size_t encoded_total_output_frames = calculate_total_output_frames(encoded_data_frames);

    const int max_codeword_bytes = static_cast<int>(kFrameHeaderBytes) + cfg.payload_bytes_per_frame;
    const int max_cells = max_codeword_bytes * 4;
    const int max_rows = required_total_rows_for_payload_cells(cfg, max_cells);
    const std::string temp_dir = "temp_frames";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    cv::Mat frame_img;
    size_t frame_count = 0;

    const std::vector<uint8_t> stream_buf =
        build_stream_header(cfg, data.size(), encoded_data_frames);

    std::vector<uint8_t> last_frame_buf;
    size_t data_offset = 0;
    const std::vector<uint8_t> padding_payload(kPaddingFramePayloadBytes, 0x00);
    for (uint32_t frame_index = 0; frame_index < encoded_data_frames; ++frame_index) {
        const size_t chunk = frame_chunks[frame_index];
        std::vector<uint8_t> source_payload;
        if (data_offset < data.size()) {
            const size_t real_chunk = std::min(chunk, data.size() - data_offset);
            source_payload = slice_payload(data, data_offset, real_chunk);
            data_offset += real_chunk;
        } else {
            source_payload = padding_payload;
        }

        std::vector<uint8_t> frame_buf =
            build_frame_codeword(frame_index, encoded_data_frames, cfg, source_payload);

        if (frame_index == 0) {
            std::vector<uint8_t> prefixed;
            prefixed.reserve(bootstrap_buf.size() + stream_buf.size() + frame_buf.size());
            prefixed.insert(prefixed.end(), bootstrap_buf.begin(), bootstrap_buf.end());
            prefixed.insert(prefixed.end(), stream_buf.begin(), stream_buf.end());
            prefixed.insert(prefixed.end(), frame_buf.begin(), frame_buf.end());
            frame_buf = std::move(prefixed);
        }

        if (frame_index + 1 == encoded_data_frames) {
            last_frame_buf = frame_buf;
        }

        render_frame(frame_img, frame_buf, cfg, max_rows);
        write_frame(frame_img, temp_dir, frame_count);
    }

    append_repeated_frame(
        last_frame_buf,
        cfg,
        max_rows,
        temp_dir,
        frame_count,
        kLastFrameRepeat);
    append_repeated_frame(
        stream_buf,
        cfg,
        max_rows,
        temp_dir,
        frame_count,
        kFinalHeaderRepeat);

    const std::string ffmpeg = find_ffmpeg_executable(argv[0]);
    const int ffmpeg_result = run_process({
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-framerate",
        std::to_string(kFixedOutputFps),
        "-i",
        temp_dir + "/frame_%d.png",
        "-c:v",
        "libx264",
        "-crf",
        "0",
        "-preset",
        "veryfast",
        "-g",
        "1",
        "-bf",
        "0",
        "-pix_fmt",
        "yuv444p",
        output_path,
    });

    fs::remove_all(temp_dir);

    if (ffmpeg_result != 0) {
        std::cerr << "Error: ffmpeg failed while encoding the video.\n";
        return static_cast<int>(ExitCode::EncodingError);
    }

    const double actual_milliseconds = 1000.0 * static_cast<double>(encoded_total_output_frames) /
        static_cast<double>(kFixedOutputFps);
    std::cout << "target duration(ms): " << std::fixed << std::setprecision(3)
              << max_milliseconds << "\n"
              << "output frames      : " << encoded_total_output_frames << "\n"
              << "output duration(ms): " << actual_milliseconds << "\n"
              << "fec rs(data/parity): " << fec_cfg.rs_data_bytes
              << "/" << fec_cfg.rs_parity_bytes << "\n"
              << "non-data frames    : "
              << (kBootstrapRepeat + kStreamHeaderRepeat + kFinalHeaderRepeat + kLastFrameRepeat) << "\n"
              << "encoded data frames: " << encoded_data_frames << "/" << full_data_frames << "\n"
              << "encoded bytes      : " << encoded_total_bytes << "/" << data.size() << "\n"
              << "padded tail bytes  : "
              << (data.size() > encoded_total_bytes ? data.size() - encoded_total_bytes : 0) << "\n";

    return static_cast<int>(ExitCode::Ok);
}
