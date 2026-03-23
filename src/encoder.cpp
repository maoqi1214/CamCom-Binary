// 实现 encode 主程序，将二进制输入编码为可视化帧并生成视频。
#include "codec.hpp"
#include "common.hpp"
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

constexpr int kFixedOutputFps = 15;
constexpr int kBootstrapRepeat = 3;
constexpr int kStreamHeaderRepeat = 3;
constexpr int kLastFrameRepeat = 3;
constexpr int kFinalHeaderRepeat = 3;
constexpr int kFixedOverheadFrames = kBootstrapRepeat + kStreamHeaderRepeat + kFinalHeaderRepeat;

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <max_milliseconds>\n"
        << "Example: " << argv0 << " payload.bin out.mp4 2000\n";
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
    const std::vector<uint8_t>& data,
    size_t offset,
    size_t chunk) {
    std::vector<uint8_t> frame_buf;
    serialize_u32(frame_buf, MAGIC);
    serialize_u8(frame_buf, FORMAT_VERSION);
    serialize_u32(frame_buf, frame_index);
    serialize_u32(frame_buf, total_frames);
    serialize_u32(frame_buf, static_cast<uint32_t>(chunk));

    const size_t checksum_pos = frame_buf.size();
    serialize_u32(frame_buf, 0);
    frame_buf.insert(frame_buf.end(), data.begin() + offset, data.begin() + offset + chunk);

    const uint32_t checksum = crc32(frame_buf.data() + checksum_pos + 4, chunk);
    frame_buf[checksum_pos + 0] = static_cast<uint8_t>(checksum & 0xFFu);
    frame_buf[checksum_pos + 1] = static_cast<uint8_t>((checksum >> 8) & 0xFFu);
    frame_buf[checksum_pos + 2] = static_cast<uint8_t>((checksum >> 16) & 0xFFu);
    frame_buf[checksum_pos + 3] = static_cast<uint8_t>((checksum >> 24) & 0xFFu);

    return frame_buf;
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
    const double max_milliseconds = std::stod(argv[3]);

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

    const size_t payload_per_frame = static_cast<size_t>(cfg.payload_bytes_per_frame);
    const uint32_t total_frames = static_cast<uint32_t>((data.size() + payload_per_frame - 1) / payload_per_frame);
    const size_t total_output_frames = calculate_total_output_frames(total_frames);
    const double output_milliseconds =
        1000.0 * static_cast<double>(total_output_frames) / static_cast<double>(kFixedOutputFps);
    if (output_milliseconds > max_milliseconds) {
        const size_t repeated_tail_frames = total_frames > 0 ? static_cast<size_t>(kLastFrameRepeat) : 0;
        std::cerr << "Error: max_milliseconds is too small.\n"
                  << "  limit           : " << std::fixed << std::setprecision(3)
                  << max_milliseconds << " ms\n"
                  << "  required minimum: " << output_milliseconds << " ms\n"
                  << "  total frames    : " << total_output_frames << "\n"
                  << "  data frames     : " << total_frames << "\n"
                  << "  overhead frames : " << (kFixedOverheadFrames + repeated_tail_frames) << "\n"
                  << "  fixed fps       : " << kFixedOutputFps << "\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    const int frame_header_bytes = 4 + 1 + 4 + 4 + 4 + 4;
    const int max_codeword_bytes = frame_header_bytes + cfg.payload_bytes_per_frame;
    const int max_cells = max_codeword_bytes * 4;
    const int max_rows = static_cast<int>((max_cells + cfg.cells_per_row - 1) / cfg.cells_per_row);
    const std::string temp_dir = "temp_frames";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    cv::Mat frame_img;
    size_t frame_count = 0;

    const std::vector<uint8_t> bootstrap_buf = build_bootstrap(cfg);
    render_frame(frame_img, bootstrap_buf, cfg, max_rows);
    for (int i = 0; i < kBootstrapRepeat; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }

    const std::vector<uint8_t> stream_buf = build_stream_header(cfg, data.size(), total_frames);
    render_frame(frame_img, stream_buf, cfg, max_rows);
    for (int i = 0; i < kStreamHeaderRepeat; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }

    std::vector<uint8_t> last_frame_buf;
    for (uint32_t frame_index = 0; frame_index < total_frames; ++frame_index) {
        const size_t offset = static_cast<size_t>(frame_index) * payload_per_frame;
        const size_t remain = data.size() - offset;
        const size_t chunk = std::min(remain, payload_per_frame);

        std::vector<uint8_t> frame_buf =
            build_frame_codeword(frame_index, total_frames, data, offset, chunk);
        if (frame_index + 1 == total_frames) {
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

    return static_cast<int>(ExitCode::Ok);
}
