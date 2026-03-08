#include "common.hpp"
#include "io.hpp"
#include "codec.hpp"
#include "rs.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace camcom;

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <fps>\n"
        << "\n"
        << "  input.bin    Path to the binary file to encode.\n"
        << "  output.mp4   Path to the output video file.\n"
        << "  fps          Frames per second (<=15 recommended).\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " payload.bin out.mp4 10\n";
}

static void serialize_u8(std::vector<uint8_t>& buf, uint8_t v) { buf.push_back(v); }
static void serialize_u32(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}
static void serialize_u64(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>((v >> (8*i)) & 0xFFu));
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string input_path   = argv[1];
    const std::string output_path  = argv[2];
    const int fps = std::stoi(argv[3]);

    if (fps <= 0 || fps > 60) {
        std::cerr << "Error: fps must be a positive integer (reasonable <=60).\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    if (!file_exists(input_path)) {
        std::cerr << "Error: input file not found: " << input_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    std::cout << "[encoder] Input : " << input_path  << "\n"
              << "[encoder] Output: " << output_path << "\n"
              << "[encoder] FPS   : " << fps << "\n";

    const auto data = read_binary_file(input_path);

    EncoderConfig cfg;
    cfg.fps = fps;

    const size_t payload_per_frame = static_cast<size_t>(cfg.payload_bytes_per_frame);
    const uint32_t total_frames = static_cast<uint32_t>((data.size() + payload_per_frame - 1) / payload_per_frame);

    std::cout << "[encoder] total bytes=" << data.size() << " frames=" << total_frames << "\n";

    cv::Mat first_img;
    cv::VideoWriter writer;

    // Write a small bootstrap frame (unprotected) that carries parameters needed to decode
    std::vector<uint8_t> bootstrap_buf;
    // magic
    serialize_u32(bootstrap_buf, MAGIC);
    // version
    serialize_u8(bootstrap_buf, FORMAT_VERSION);
    // rs_nsym
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.rs_nsym));
    // cell_size
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.cell_size));
    // cells_per_row
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.cells_per_row));
    // payload_bytes_per_frame
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    // fps
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.fps));
    // reference_block_size
    serialize_u32(bootstrap_buf, static_cast<uint32_t>(cfg.reference_block_size));

    render_frame(first_img, bootstrap_buf, cfg);
    const int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
    writer.open(output_path, fourcc, cfg.fps, first_img.size());
    if (!writer.isOpened()) {
        std::cerr << "Error: failed to open VideoWriter for " << output_path << "\n";
        return static_cast<int>(ExitCode::EncodingError);
    }

    // Repeat bootstrap frame a few times to improve robustness
    const int BOOTSTRAP_REPEAT = 3;
    cv::Mat black0 = first_img.clone(); black0.setTo(cv::Scalar(0,0,0));
    for (int i = 0; i < BOOTSTRAP_REPEAT; ++i) {
        writer.write(first_img);
        writer.write(black0);
    }

    // Now write the StreamHeader protected by RS parity
    std::vector<uint8_t> stream_buf;
    // magic
    serialize_u32(stream_buf, MAGIC);
    // version
    serialize_u8(stream_buf, FORMAT_VERSION);
    // total_data_bytes (u64)
    serialize_u64(stream_buf, static_cast<uint64_t>(data.size()));
    // encoding (u8)
    serialize_u8(stream_buf, static_cast<uint8_t>(Encoding::Binary));
    // fps
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.fps));
    // cell_size
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.cell_size));
    // rs_nsym
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.rs_nsym));
    // payload_bytes_per_frame
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    // cells_per_row
    serialize_u32(stream_buf, static_cast<uint32_t>(cfg.cells_per_row));
    // total_frames
    serialize_u32(stream_buf, total_frames);

    // append RS parity to stream header
    std::vector<uint8_t> stream_parity = rs::encode(stream_buf, cfg.rs_nsym);
    stream_buf.insert(stream_buf.end(), stream_parity.begin(), stream_parity.end());
    render_frame(first_img, stream_buf, cfg);
    // Repeat RS-protected StreamHeader several times to ensure decoder captures it
    const int STREAMHDR_REPEAT = 3;
    for (int i = 0; i < STREAMHDR_REPEAT; ++i) {
        writer.write(first_img);
        writer.write(black0);
    }

    for (uint32_t fi = 0; fi < total_frames; ++fi) {
        const size_t offset = static_cast<size_t>(fi) * payload_per_frame;
        const size_t remain = (offset < data.size()) ? (data.size() - offset) : 0;
        const size_t chunk = std::min(remain, payload_per_frame);

        // Build frame buffer: FrameHeader + payload
        std::vector<uint8_t> frame_buf;
        // magic
        serialize_u32(frame_buf, MAGIC);
        // version
        frame_buf.push_back(FORMAT_VERSION);
        // frame_index
        serialize_u32(frame_buf, fi);
        // total_frames
        serialize_u32(frame_buf, total_frames);
        // payload_bytes
        serialize_u32(frame_buf, static_cast<uint32_t>(chunk));
        // placeholder checksum (will fill after payload appended)
        const size_t checksum_pos = frame_buf.size();
        serialize_u32(frame_buf, 0);

        // append payload bytes
        frame_buf.insert(frame_buf.end(), data.begin() + offset, data.begin() + offset + chunk);

        // compute checksum over payload portion
        const uint32_t checksum = crc32(frame_buf.data() + checksum_pos + 4, chunk);
        // write checksum into buffer (little-endian)
        frame_buf[checksum_pos + 0] = static_cast<uint8_t>(checksum & 0xFFu);
        frame_buf[checksum_pos + 1] = static_cast<uint8_t>((checksum >> 8) & 0xFFu);
        frame_buf[checksum_pos + 2] = static_cast<uint8_t>((checksum >> 16) & 0xFFu);
        frame_buf[checksum_pos + 3] = static_cast<uint8_t>((checksum >> 24) & 0xFFu);

        // Append RS parity bytes
        std::vector<uint8_t> parity = rs::encode(frame_buf, cfg.rs_nsym);
        frame_buf.insert(frame_buf.end(), parity.begin(), parity.end());

        // Render frame image
        render_frame(first_img, frame_buf, cfg);
        writer.write(first_img);

        // insert a black transition frame to reduce motion blur artifacts
        cv::Mat black = first_img.clone();
        black.setTo(cv::Scalar(0,0,0));
        writer.write(black);
    }

    writer.release();

    std::cout << "[encoder] Done.\n";
    return static_cast<int>(ExitCode::Ok);
}
