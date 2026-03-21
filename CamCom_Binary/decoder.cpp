#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <filesystem> // C++17, 用于创建临时目录
#include <thread>
#include <chrono>

// Include local headers
#include "Frame.h"    // Changed from Frame108.hpp
#include "common.h"      // Changed from common.hpp, contains camcom::ExitCode
#include "io.h"          // Changed from io.hpp, contains file operations

// ---------------------------------------------------------------------------
// Configuration Constants
// ---------------------------------------------------------------------------

// 根据 Frame108 结构计算每帧最大数据载荷，使用四色单元编码（2 bit / cell）
constexpr int DATA_BYTES_PER_FRAME = Frame108::DATA_BYTES_CAPACITY;
constexpr int DATA_BITS_PER_FRAME = DATA_BYTES_PER_FRAME * 8;

// CRC32 简单实现 (Polynomial 0xEDB88320)
static uint32_t calculate_crc32(const std::vector<uint8_t>& data, uint32_t frame_id) {
    uint32_t crc = 0xFFFFFFFF;
    // 将 frame_id 也纳入校验范围 (大端序)
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
        << "Usage: " << argv0 << " <recorded.mp4> <output.bin> <validity_mask.bin>\n"
        << "\n"
        << "  recorded.mp4      Path to the video file captured by the camera.\n"
        << "  output.bin        Path where the decoded binary data will be written.\n"
        << "  validity_mask.bin Path where the per-bit validity mask will be written.\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " captured.mp4 recovered.bin mask.bin\n";
}

// ---------------------------------------------------------------------------
// Helper Functions for Command Line Execution
// ---------------------------------------------------------------------------

static int execute_command(const std::string& command) {
    std::cout << "[decoder] Executing command: " << command << "\n";
    return std::system(command.c_str());
}

// ---------------------------------------------------------------------------
// decoder
// ---------------------------------------------------------------------------

int run_decoder(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    const std::string video_path = argv[1];
    const std::string output_path = argv[2];
    const std::string mask_path = argv[3];

    if (!file_exists(video_path)) { // Removed camcom::
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    std::cout << "[decoder] Video : " << video_path << "\n"
        << "[decoder] Output: " << output_path << "\n"
        << "[decoder] Mask  : " << mask_path << "\n"
        << "[decoder] Starting FFmpeg demuxing via command line...\n";

    // --- 1. Create temporary directory for extracted frames ---
    std::string temp_dir = "temp_extracted_frames_" + std::to_string(std::time(nullptr));
    if (!std::filesystem::create_directory(temp_dir)) {
        std::cerr << "Error: Could not create temporary directory: " << temp_dir << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }
    std::cout << "[decoder] Created temporary directory: " << temp_dir << "\n";

    // --- 2. Extract frames using FFmpeg command line ---
    std::string extract_cmd = "ffmpeg -y -i \"" + video_path + "\" -start_number 0 \"" + temp_dir + "/frame_%06d.png\"";
    if (execute_command(extract_cmd) != 0) {
        std::cerr << "Error: FFmpeg failed to extract frames.\n";
        std::filesystem::remove_all(temp_dir);
        return static_cast<int>(camcom::ExitCode::IoError);
    }
    std::cout << "[decoder] Frames extracted successfully.\n";

    // --- 3. List all extracted frames ---
    std::vector<std::string> frame_files;
    for (const auto& entry : std::filesystem::directory_iterator(temp_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".png") {
            frame_files.push_back(entry.path().string());
        }
    }
    std::sort(frame_files.begin(), frame_files.end()); // Ensure order
    std::cout << "[decoder] Found " << frame_files.size() << " frames to process.\n";

    // --- 4. Decode Loop & Data Reconstruction ---

    // Map to store received frames: FrameID -> Data
    // We use a map to handle out-of-order frames naturally
    std::map<uint32_t, std::vector<uint8_t>> received_frames;

    uint32_t max_frame_id = 0;
    int successful_frames = 0;
    int failed_frames = 0;
    int duplicate_frames = 0;

    std::cout << "[decoder] Processing frames...\n";

    for (size_t i = 0; i < frame_files.size(); ++i) {
        // Load frame image using OpenCV
        cv::Mat img = cv::imread(frame_files[i]);
        if (img.empty()) {
            std::cerr << "[warn] Could not load image: " << frame_files[i] << "\n";
            failed_frames++;
            continue;
        }

        // --- Process Frame ---
        Frame108 decoded_frame_struct;
        // Try robust decoding (Thresholding + Perspective Correction + Parsing)
        if (Frame108::fromImageRobust(img, decoded_frame_struct, DATA_BITS_PER_FRAME)) {
            uint32_t fid = decoded_frame_struct.getFrameId();
            uint32_t stored_crc = decoded_frame_struct.getChecksum();
            std::vector<uint8_t> payload = decoded_frame_struct.getData(DATA_BITS_PER_FRAME);

            // Verify CRC
            uint32_t calc_crc = calculate_crc32(payload, fid);

            if (calc_crc == stored_crc) {
                // Check for duplicates
                if (received_frames.find(fid) == received_frames.end()) {
                    received_frames[fid] = payload;
                    if (fid > max_frame_id) max_frame_id = fid;
                    successful_frames++;

                    if (successful_frames % 50 == 0) {
                        std::cout << "[decoder] Progress: Received " << successful_frames
                            << " valid frames (Max ID: " << max_frame_id << ", Current: " << fid << ")\n";
                    }
                }
                else {
                    duplicate_frames++;
                }
            }
            else {
                failed_frames++;
                // Optional: Log specific bad CRC
                // std::cout << "[warn] Frame " << fid << " CRC mismatch.\n";
            }
        }
        else {
            failed_frames++;
        }

        // Simple progress indicator
        if (i > 0 && i % 100 == 0) {
            std::cout << "[decoder] Processed " << i << " frames out of " << frame_files.size() << "\n";
        }
    }

    std::cout << "\n[decoder] Decoding finished.\n";
    std::cout << "  Total frames processed: " << frame_files.size() << "\n";
    std::cout << "  Valid frames   : " << successful_frames << "\n";
    std::cout << "  Duplicate frames: " << duplicate_frames << "\n";
    std::cout << "  Failed/Invalid : " << failed_frames << "\n";

    // --- 5. Reassemble Data ---

    // Calculate total expected size (assuming contiguous from 0 to max_frame_id)
    // Note: If frames are missing at the end, we might over-allocate, but the mask will show 0s.
    size_t total_bytes = (max_frame_id + 1) * DATA_BYTES_PER_FRAME;

    std::vector<uint8_t> final_data(total_bytes, 0x00);
    std::vector<uint8_t> validity_mask(total_bytes, 0x00); // 0xFF = valid, 0x00 = invalid

    uint32_t missing_count = 0;
    for (uint32_t i = 0; i <= max_frame_id; ++i) {
        auto it = received_frames.find(i);
        if (it != received_frames.end()) {
            size_t offset = i * DATA_BYTES_PER_FRAME;

            // Determine actual bytes to copy (last frame might be partial if we knew file size,
            // but here we assume full blocks or rely on upper layer protocol to trim)
            // For safety, we copy what we have.
            size_t copy_len = std::min(it->second.size(), static_cast<size_t>(DATA_BYTES_PER_FRAME));

            std::memcpy(final_data.data() + offset, it->second.data(), copy_len);

            // Mark as valid in mask
            std::fill(validity_mask.begin() + offset, validity_mask.begin() + offset + copy_len, 0xFF);
        }
        else {
            missing_count++;
            // Leave as 0x00 in data and mask
        }
    }

    if (missing_count > 0) {
        std::cout << "[warn] Missing " << missing_count << " frames in sequence. Output file may be corrupted.\n";
    }

    // --- 6. Write Output ---

    try {
        write_binary_file(output_path, final_data); // Removed camcom::
        write_binary_file(mask_path, validity_mask); // Removed camcom::
        std::cout << "[decoder] Successfully wrote output to: " << output_path << "\n";
        std::cout << "[decoder] Successfully wrote mask to: " << mask_path << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error writing files: " << e.what() << "\n";
        // Clean up temp dir before exiting
        std::filesystem::remove_all(temp_dir);
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    // --- 7. Cleanup Temporary Directory ---
    std::cout << "[decoder] Cleaning up temporary directory...\n";
    std::filesystem::remove_all(temp_dir);
    std::cout << "[decoder] Done.\n";

    return static_cast<int>(camcom::ExitCode::Ok);
}