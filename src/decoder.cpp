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

using namespace camcom;

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.mp4> <output.bin>\n"
        << "\n"
        << "  input.mp4    Path to the video file to decode.\n"
        << "  output.bin   Path where the decoded binary data will be written.\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " input.mp4 output.bin\n";
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

int main(int argc, char* argv[]) {
    std::cout << "[decoder] Starting...\n";
    std::cout << "[decoder] argc: " << argc << "\n";
    for (int i = 0; i < argc; ++i) {
        std::cout << "[decoder] argv[" << i << "]: " << argv[i] << "\n";
    }

    if (argc != 3) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string video_path = argv[1];
    const std::string output_path = argv[2];

    std::cout << "[decoder] Input video: " << video_path << "\n";
    std::cout << "[decoder] Output file: " << output_path << "\n";

    if (!file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }
    std::cout << "[decoder] Video file exists\n";

    EncoderConfig cfg;

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: failed to open video: " << video_path << "\n";
        return static_cast<int>(ExitCode::DecodingError);
    }
    std::cout << "[decoder] Video opened successfully\n";

    std::vector<std::vector<uint8_t>> frames_buffer;
    uint32_t expected_total_frames = 0;
    bool have_bootstrap = false;
    bool have_stream_header = false;

    uint32_t stream_rs_nsym = 0;

    cv::Mat frame;
    int frame_count = 0;
    while (cap.read(frame)) {
        frame_count++;
        std::cout << "[decoder] Processing frame " << frame_count << "\n";

        // Skip black frames
        cv::Scalar mean = cv::mean(frame);
        if (mean[0] < 10 && mean[1] < 10 && mean[2] < 10) {
            std::cout << "[decoder] Skipping black frame\n";
            continue;
        }

        // Sample the frame
        std::vector<uint8_t> sample;
        if (!sample_frame(frame, sample, cfg)) {
            std::cout << "[decoder] Failed to sample frame\n";
            continue;
        }
        std::cout << "[decoder] Sampled frame, sample size: " << sample.size() << "\n";

        // If we haven't seen any bootstrap, check whether this frame is the bootstrap (unprotected)
        if (!have_bootstrap) {
            // minimal size for bootstrap: u32 magic + u8 version + 5*u32 = 4+1+20 = 25
            if (sample.size() >= 28) {
                const uint8_t* p = sample.data();
                uint32_t magic = read_u32_le(p); p += 4;
                uint8_t version = *p; p += 1;
                std::cout << "[decoder] Checking bootstrap: magic=0x" << std::hex << magic << ", version=" << std::dec << (int)version << "\n";
                if (magic == MAGIC && version == FORMAT_VERSION) {
                    // parse bootstrap fields in same order as encoder
                    uint32_t rs_nsym = read_u32_le(p); p += 4;
                    uint32_t cell_size = read_u32_le(p); p += 4;
                    uint32_t cells_per_row = read_u32_le(p); p += 4;
                    uint32_t payload_per = read_u32_le(p); p += 4;
                    uint32_t fps = read_u32_le(p); p += 4;
                    uint32_t reference_block_size = read_u32_le(p); p += 4;

                    // apply to cfg
                    cfg.cell_size = static_cast<int>(cell_size);
                    cfg.fps = static_cast<int>(fps);
                    cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
                    cfg.cells_per_row = static_cast<int>(cells_per_row);
                    cfg.rs_nsym = static_cast<int>(rs_nsym);
                    cfg.reference_block_size = static_cast<int>(reference_block_size);
                    stream_rs_nsym = rs_nsym;
                    have_bootstrap = true;
                    std::cout << "[decoder] Found bootstrap frame with config: cell_size=" << cell_size << ", cells_per_row=" << cells_per_row << "\n";
                    continue;
                }
            }
            // not bootstrap: skip
            std::cout << "[decoder] Not a bootstrap frame\n";
            continue;
        }

        // If we have bootstrap but not yet stream header, then this frame should be RS-protected StreamHeader
        if (have_bootstrap && !have_stream_header) {
            std::vector<uint8_t> codeword = sample;
            if (stream_rs_nsym > 0) {
                std::cout << "[decoder] Decoding RS for stream header\n";
                if (!rs::decode(codeword, static_cast<int>(stream_rs_nsym))) {
                    std::cout << "[decoder] RS decode failed for stream header\n";
                    continue;
                }
            }
            const uint8_t* p = codeword.data();
            uint32_t magic = read_u32_le(p); p += 4;
            uint8_t version = *p; p += 1;
            std::cout << "[decoder] Checking stream header: magic=0x" << std::hex << magic << ", version=" << std::dec << (int)version << "\n";
            if (!(magic == MAGIC && version == FORMAT_VERSION)) {
                std::cout << "[decoder] Not a stream header\n";
                continue;
            }
            // parse StreamHeader fields
            uint64_t total_data = read_u64_le(p); p += 8;
            uint8_t encoding = *p; p += 1;
            uint32_t fps = read_u32_le(p); p += 4;
            uint32_t cell_size = read_u32_le(p); p += 4;
            uint32_t rs_nsym = read_u32_le(p); p += 4;
            uint32_t payload_per = read_u32_le(p); p += 4;
            uint32_t cells_per_row = read_u32_le(p); p += 4;
            uint32_t total_frames_hdr = read_u32_le(p); p += 4;

            // apply to cfg
            cfg.cell_size = static_cast<int>(cell_size);
            cfg.fps = static_cast<int>(fps);
            cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
            cfg.cells_per_row = static_cast<int>(cells_per_row);
            cfg.rs_nsym = static_cast<int>(rs_nsym);
            stream_rs_nsym = rs_nsym;
            expected_total_frames = total_frames_hdr;
            frames_buffer.resize(expected_total_frames);
            have_stream_header = true;
            std::cout << "[decoder] Found stream header: total_data_bytes=" << total_data << ", total_frames=" << total_frames_hdr << "\n";
            continue;
        }

        // For data frames, sample contains codeword = message + parity
        std::vector<uint8_t> codeword = sample;
        if (stream_rs_nsym > 0) {
            std::cout << "[decoder] Decoding RS for data frame\n";
            if (!rs::decode(codeword, static_cast<int>(stream_rs_nsym))) {
                std::cout << "[decoder] RS decode failed for data frame\n";
                continue;
            }
        }

        // after RS decode, parse FrameHeader then payload
        const uint8_t* p = codeword.data();
        FrameHeader hdr;
        hdr.magic = read_u32_le(p); p += 4;
        hdr.version = *p; p += 1;
        hdr.frame_index = read_u32_le(p); p += 4;
        hdr.total_frames = read_u32_le(p); p += 4;
        hdr.payload_bytes = read_u32_le(p); p += 4;
        hdr.checksum = read_u32_le(p); p += 4;

        std::cout << "[decoder] Checking data frame: magic=0x" << std::hex << hdr.magic << ", version=" << std::dec << (int)hdr.version << ", frame_index=" << hdr.frame_index << "\n";
        if (hdr.magic != MAGIC || hdr.version != FORMAT_VERSION) {
            std::cout << "[decoder] Not a data frame\n";
            continue;
        }

        size_t payload_len = hdr.payload_bytes;
        if (p + payload_len > codeword.data() + codeword.size()) {
            std::cout << "[decoder] Payload length exceeds codeword size\n";
            continue;
        }

        const uint8_t* payload_ptr = p;
        const uint32_t calc = crc32(payload_ptr, payload_len);
        std::cout << "[decoder] Checksum: expected=0x" << std::hex << hdr.checksum << ", calculated=0x" << calc << "\n";
        if (calc != hdr.checksum) {
            std::cout << "[decoder] Checksum mismatch\n";
            continue;
        }

        // accept
        if (hdr.total_frames > 0 && expected_total_frames == 0) {
            expected_total_frames = hdr.total_frames;
            frames_buffer.resize(expected_total_frames);
        }

        if (hdr.frame_index < frames_buffer.size()) {
            frames_buffer[hdr.frame_index] = std::vector<uint8_t>(payload_ptr, payload_ptr + payload_len);
            std::cout << "[decoder] Decoded frame " << hdr.frame_index << " of " << hdr.total_frames << "\n";
        }
    }

    // reassemble
    std::vector<uint8_t> recovered;
    if (expected_total_frames == 0) {
        std::cerr << "Warning: no frames decoded successfully." << std::endl;
    }
    else {
        for (uint32_t i = 0; i < expected_total_frames; ++i) {
            if (i < frames_buffer.size() && !frames_buffer[i].empty()) {
                recovered.insert(recovered.end(), frames_buffer[i].begin(), frames_buffer[i].end());
            }
            else {
                // missing frame -> skip (could insert zeros)
                std::cerr << "Warning: missing frame " << i << "\n";
            }
        }
    }

    // write output
    try {
        std::cout << "[decoder] Writing output file: " << output_path << "\n";
        write_binary_file(output_path, recovered);
        std::cout << "[decoder] Output file written successfully\n";
    }
    catch (const std::exception& ex) {
        std::cerr << "Error writing output: " << ex.what() << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    std::cout << "[decoder] Done. Recovered bytes=" << recovered.size() << "\n";
    return static_cast<int>(ExitCode::Ok);
}