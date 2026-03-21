#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <filesystem> // C++17, 用于创建临时目录
#include <thread>
#include <chrono>

// Include local headers
#include "Frame.h"    // Changed from Frame108.hpp
#include "common.h"      // Changed from common.hpp, contains camcom::crc32
#include "io.h"          // Changed from io.hpp, contains file operations

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

// 目标视频分辨率 (放大后，便于相机对焦和抗摩尔纹)
constexpr int VIDEO_WIDTH = 640;
constexpr int VIDEO_HEIGHT = 480;

// 目标帧率 (FPS)
// 20-24 FPS 是屏幕 - 相机通信的甜蜜点：
// 太低 -> 传输慢；太高 -> 相机曝光不足或滚动快门效应严重
constexpr int TARGET_FPS = 20;

// 每帧最大数据载荷，使用四色单元编码（2 bit / cell）
constexpr int DATA_BYTES_PER_FRAME = Frame108::DATA_BYTES_CAPACITY;

// CRC32 简单实现 (Polynomial 0xEDB88320)
static uint32_t calculate_crc32(const std::vector<uint8_t>& data, uint32_t frame_id) {
    uint32_t crc = 0xFFFFFFFF;
    uint8_t id_bytes[4];
    id_bytes[0] = (frame_id >> 24) & 0xFF;
    id_bytes[1] = (frame_id >> 16) & 0xFF;
    id_bytes[2] = (frame_id >> 8) & 0xFF;
    id_bytes[3] = frame_id & 0xFF;

    auto update = [&](const uint8_t* buf, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            crc ^= buf[i];
            for (int j = 0; j < 8; ++j) {
                crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
            }
        }
        };

    update(id_bytes, 4);
    update(data.data(), data.size());

    return crc ^ 0xFFFFFFFF;
}

// ---------------------------------------------------------------------------
// Usage
// ---------------------------------------------------------------------------

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <duration_ms>\n"
        << "\n"
        << "  input.bin    Path to the binary file to encode.\n"
        << "  output.mp4   Path to the output video file.\n"
        << "  duration_ms  Total video duration in milliseconds.\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " payload.bin out.mp4 5000\n";
}

// ---------------------------------------------------------------------------
// Helper Functions for Command Line Execution
// ---------------------------------------------------------------------------

static int execute_command(const std::string& command) {
    std::cout << "[encoder] Executing command: " << command << "\n";
    return std::system(command.c_str());
}

// ---------------------------------------------------------------------------
// encoder
// ---------------------------------------------------------------------------

int run_encoder(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    const long        duration_ms = std::stol(argv[3]);

    if (duration_ms <= 0) {
        std::cerr << "Error: duration_ms must be a positive integer.\n";
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    if (!file_exists(input_path)) { // Removed camcom::
        std::cerr << "Error: input file not found: " << input_path << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    std::cout << "[encoder] Input : " << input_path << "\n"
        << "[encoder] Output: " << output_path << "\n"
        << "[encoder] Duration (ms): " << duration_ms << "\n";

    // --- 1. Read Input Data ---
    std::vector<uint8_t> input_data;
    try {
        input_data = read_binary_file(input_path); // Removed camcom::
    }
    catch (const std::exception& e) {
        std::cerr << "Error reading input file: " << e.what() << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    if (input_data.empty()) {
        std::cerr << "Error: Input file is empty.\n";
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    std::cout << "[encoder] Input size: " << input_data.size() << " bytes\n";

    // --- 2. Calculate Frame Parameters ---

    // 计算总帧数：基于时长和目标 FPS
    double duration_sec = duration_ms / 1000.0;
    int total_frames = static_cast<int>(std::ceil(duration_sec * TARGET_FPS));

    // 确保至少有 1 帧
    if (total_frames < 1) total_frames = 1;

    // 计算每帧需要承载的字节数
    // 如果数据量很小，每帧可能只需要很少的字节，但我们仍然发送满帧（填充0）以保持时序稳定
    // 如果数据量很大，我们需要检查是否能在给定时间内传完
    size_t bytes_per_frame = (input_data.size() + total_frames - 1) / total_frames;

    if (bytes_per_frame > DATA_BYTES_PER_FRAME) {
        std::cerr << "Error: Data too large to transmit in " << duration_ms << "ms at " << TARGET_FPS << " FPS.\n";
        std::cerr << "Required capacity per frame: " << bytes_per_frame << " bytes\n";
        std::cerr << "Max capacity per frame: " << DATA_BYTES_PER_FRAME << " bytes\n";
        std::cerr << "Suggestion: Increase duration_ms or reduce input file size.\n";
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    std::cout << "[encoder] Total frames: " << total_frames << "\n";
    std::cout << "[encoder] Payload per frame: " << bytes_per_frame << " bytes\n";

    // --- 3. Create temporary directory for generated frames ---
    std::string temp_dir = "temp_generated_frames_" + std::to_string(std::time(nullptr));
    if (!std::filesystem::create_directory(temp_dir)) {
        std::cerr << "Error: Could not create temporary directory: " << temp_dir << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }
    std::cout << "[encoder] Created temporary directory: " << temp_dir << "\n";

    // --- 4. Generate Frames using Frame108 Logic ---
    std::cout << "[encoder] Generating frames...\n";

    size_t data_offset = 0;
    int frames_generated = 0;

    for (int i = 0; i < total_frames; ++i) {
        // A. Prepare Payload
        std::vector<uint8_t> chunk;
        size_t remaining = input_data.size() - data_offset;
        size_t to_copy = std::min(remaining, bytes_per_frame);

        chunk.resize(to_copy);
        if (to_copy > 0) {
            std::memcpy(chunk.data(), input_data.data() + data_offset, to_copy);
        }
        // 如果不足 bytes_per_frame，chunk 会自动保持较小尺寸，Frame108 内部会处理填充

        // B. Calculate CRC
        uint32_t crc = calculate_crc32(chunk, static_cast<uint32_t>(i));

        // C. Render Frame108
        Frame108 f108;
        f108.encode(static_cast<uint32_t>(i), chunk, crc);
        cv::Mat small_img = f108.toImage(); // 108x108 CV_8UC3 (BGR)

        // D. Resize the frame using OpenCV
        cv::Mat resized_img;
        cv::resize(small_img, resized_img, cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT), 0, 0, cv::INTER_NEAREST); // Use nearest neighbor to preserve sharp color boundaries

        // E. Save the resized frame to temporary directory
        std::string frame_filename = temp_dir + "/frame_" + std::to_string(i) + ".png";
        if (!cv::imwrite(frame_filename, resized_img)) {
            std::cerr << "[error] Could not save frame " << i << " to " << frame_filename << "\n";
            // Clean up temp dir before exiting
            std::filesystem::remove_all(temp_dir);
            return static_cast<int>(camcom::ExitCode::IoError);
        }

        // Update offset
        data_offset += to_copy;
        frames_generated++;

        if (frames_generated % 50 == 0) {
            std::cout << "[encoder] Progress: " << frames_generated << "/" << total_frames << " frames\n";
        }
    }

    std::cout << "[encoder] Frame generation complete. Total frames: " << frames_generated << "\n";

    // --- 5. Encode Video using FFmpeg command line ---
    std::cout << "[encoder] Starting FFmpeg encoding...\n";

    // FFmpeg command to convert the image sequence into a video
    // -framerate: Input framerate
    // -i: Input pattern (frame_%01d.png means frame_0.png, frame_1.png, ...)
    // -c:v libx264: Use H.264 codec
    // -pix_fmt yuv444p: Preserve chroma for colored symbols
    // -crf 8: High quality (lower number = higher quality)
    // -preset ultrafast: Fast encoding
    std::string encode_cmd = "ffmpeg -y -r " + std::to_string(TARGET_FPS) +
        " -start_number 0 -i \"" + temp_dir + "/frame_%d.png\" " +
        "-c:v libx264 -pix_fmt yuv444p -crf 8 -preset ultrafast " +
        "-vf fps=" + std::to_string(TARGET_FPS) + " \"" + output_path + "\"";

    if (execute_command(encode_cmd) != 0) {
        std::cerr << "Error: FFmpeg failed to encode video.\n";
        // Clean up temp dir before exiting
        std::filesystem::remove_all(temp_dir);
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    std::cout << "[encoder] Video encoding complete.\n";
    std::cout << "[encoder] Output saved to: " << output_path << "\n";

    // --- 6. Cleanup Temporary Directory ---
    //std::cout << "[encoder] Cleaning up temporary directory...\n";
    //std::filesystem::remove_all(temp_dir);
    //std::cout << "[encoder] Done.\n";

    return static_cast<int>(camcom::ExitCode::Ok);
}