// Encoder entry point for turning binary data into video frames.
#include "codec.hpp"
#include "common.hpp"
#include "ffmpeg.hpp"
#include "io.hpp"
#include "rs.hpp"

#include <clocale>
#include <filesystem>
#include <opencv2/opencv.hpp>
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

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <fps>\n"
        << "Example: " << argv0 << " payload.bin out.mp4 15\n";
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
    serialize_u32(buf, static_cast<uint32_t>(cfg.rs_nsym));
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
    serialize_u32(buf, static_cast<uint32_t>(cfg.rs_nsym));
    serialize_u32(buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(buf, total_frames);

    std::vector<uint8_t> parity = rs::encode(buf, cfg.rs_nsym);
    buf.insert(buf.end(), parity.begin(), parity.end());
    return buf;
}

std::vector<uint8_t> build_frame_codeword(
    uint32_t frame_index,
    uint32_t total_frames,
    const std::vector<uint8_t>& data,
    size_t offset,
    size_t chunk,
    const EncoderConfig& cfg) {
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

    std::vector<uint8_t> parity = rs::encode(frame_buf, cfg.rs_nsym);
    frame_buf.insert(frame_buf.end(), parity.begin(), parity.end());
    return frame_buf;
}

void write_frame(const cv::Mat& img, const std::string& temp_dir, size_t& frame_count) {
    cv::imwrite(temp_dir + "/frame_" + std::to_string(frame_count++) + ".png", img);
}

cv::Mat fit_to_video_canvas(const cv::Mat& src, int canvas_w, int canvas_h) {
    cv::Mat dst(canvas_h, canvas_w, CV_8UC3, cv::Scalar(128, 128, 128));
    const int w = std::min(src.cols, canvas_w);
    const int h = std::min(src.rows, canvas_h);
    src(cv::Rect(0, 0, w, h)).copyTo(dst(cv::Rect(0, 0, w, h)));
    return dst;
}

void append_repeated_frame(
    const std::vector<uint8_t>& payload,
    const EncoderConfig& cfg,
    int video_w,
    int video_h,
    const std::string& temp_dir,
    size_t& frame_count,
    int repeat) {
    if (repeat <= 0 || payload.empty()) {
        return;
    }

    cv::Mat frame_img;
    render_frame(frame_img, payload, cfg);
    frame_img = fit_to_video_canvas(frame_img, video_w, video_h);
    for (int i = 0; i < repeat; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }
}

} // namespace

int main(int argc, char* argv[]) {
    init_console_utf8();

    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    const int fps = std::stoi(argv[3]);

    if (fps <= 0 || fps > 60) {
        std::cerr << "Error: fps must be in range 1..60.\n";
        return static_cast<int>(ExitCode::BadArgs);
    }
    if (!file_exists(input_path)) {
        std::cerr << "Error: input file not found: " << input_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    const auto data = read_binary_file(input_path);

    EncoderConfig cfg;
    cfg.fps = fps;
    cfg.cell_size = 16;
    cfg.cells_per_row = 108;
    cfg.payload_bytes_per_frame = 2875;

    const size_t payload_per_frame = static_cast<size_t>(cfg.payload_bytes_per_frame);
    const uint32_t total_frames = static_cast<uint32_t>((data.size() + payload_per_frame - 1) / payload_per_frame);

    const int frame_header_bytes = 4 + 1 + 4 + 4 + 4 + 4;
    const int max_codeword_bytes = frame_header_bytes + cfg.payload_bytes_per_frame + cfg.rs_nsym;
    const int max_cells = max_codeword_bytes * 4;
    const int max_rows = static_cast<int>((max_cells + cfg.cells_per_row - 1) / cfg.cells_per_row);
    const int video_w = (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS) * cfg.cell_size;
    const int video_h = (max_rows + 2 * FINDER_MARKER_CELLS) * cfg.cell_size;

    const std::string temp_dir = "temp_frames";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    cv::Mat frame_img;
    size_t frame_count = 0;

    const std::vector<uint8_t> bootstrap_buf = build_bootstrap(cfg);
    render_frame(frame_img, bootstrap_buf, cfg);
    frame_img = fit_to_video_canvas(frame_img, video_w, video_h);
    for (int i = 0; i < 3; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }

    const std::vector<uint8_t> stream_buf = build_stream_header(cfg, data.size(), total_frames);
    render_frame(frame_img, stream_buf, cfg);
    frame_img = fit_to_video_canvas(frame_img, video_w, video_h);
    for (int i = 0; i < 3; ++i) {
        write_frame(frame_img, temp_dir, frame_count);
    }

    std::vector<uint8_t> last_frame_buf;
    for (uint32_t frame_index = 0; frame_index < total_frames; ++frame_index) {
        const size_t offset = static_cast<size_t>(frame_index) * payload_per_frame;
        const size_t remain = data.size() - offset;
        const size_t chunk = std::min(remain, payload_per_frame);

        std::vector<uint8_t> frame_buf =
            build_frame_codeword(frame_index, total_frames, data, offset, chunk, cfg);
        if (frame_index + 1 == total_frames) {
            last_frame_buf = frame_buf;
        }

        render_frame(frame_img, frame_buf, cfg);
        frame_img = fit_to_video_canvas(frame_img, video_w, video_h);
        write_frame(frame_img, temp_dir, frame_count);
    }

    append_repeated_frame(last_frame_buf, cfg, video_w, video_h, temp_dir, frame_count, 3);
    append_repeated_frame(stream_buf, cfg, video_w, video_h, temp_dir, frame_count, 3);

    const std::string ffmpeg = find_ffmpeg_executable(argv[0]);
    const int ffmpeg_result = run_process({
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-framerate",
        std::to_string(fps),
        "-i",
        temp_dir + "/frame_%d.png",
        "-c:v",
        "libx264",
        "-crf",
        "12",
        "-preset",
        "slow",
        "-pix_fmt",
        "yuv420p",
        output_path,
    });

    fs::remove_all(temp_dir);

    if (ffmpeg_result != 0) {
        std::cerr << "Error: ffmpeg failed while encoding the video.\n";
        return static_cast<int>(ExitCode::EncodingError);
    }

    return static_cast<int>(ExitCode::Ok);
}
