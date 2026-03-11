#include "common.hpp"
#include "io.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

/// FourCC codec identifier used for the output video container (MPEG-4, 'mp4v').
/// Computed at runtime since cv::VideoWriter::fourcc is not constexpr.
static const int VIDEO_FOURCC = cv::VideoWriter::fourcc('m', 'p', '4', 'v');

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
// Bit-level helpers
// ---------------------------------------------------------------------------

/// Convert a byte buffer to a bit vector (MSB first per byte).
static std::vector<uint8_t> bytes_to_bits(const uint8_t* data, std::size_t len) {
    std::vector<uint8_t> bits;
    bits.reserve(len * 8);
    for (std::size_t i = 0; i < len; ++i)
        for (int b = 7; b >= 0; --b)
            bits.push_back((data[i] >> b) & 1u);
    return bits;
}

// ---------------------------------------------------------------------------
// Frame rendering
// ---------------------------------------------------------------------------

/// Render a bit vector as a grid of black (0) / white (1) cells.
/// Cells are filled left-to-right, top-to-bottom; unfilled cells are black.
static cv::Mat render_frame(const std::vector<uint8_t>& bits,
                             int cell_size, int width, int height) {
    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    const int cols     = width  / cell_size;
    const int rows     = height / cell_size;
    const int num_bits = static_cast<int>(bits.size());
    int bit_idx = 0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const uint8_t v = (bit_idx < num_bits)
                              ? (bits[bit_idx++] ? 255 : 0) : 0;
            cv::rectangle(frame,
                          cv::Rect(c * cell_size, r * cell_size, cell_size, cell_size),
                          cv::Scalar(v, v, v),
                          cv::FILLED);
        }
    }
    return frame;
}

// ---------------------------------------------------------------------------
// Capacity helpers
// ---------------------------------------------------------------------------

/// Payload bytes that fit in frame 0 (which also carries the stream header).
static int payload_capacity_frame0(int cells_per_frame) {
    const int overhead_bits = static_cast<int>(
        (camcom::STREAM_HEADER_SIZE + camcom::FRAME_HEADER_SIZE) * 8);
    const int available = cells_per_frame - overhead_bits;
    return (available > 0) ? (available / 8) : 0;
}

/// Payload bytes that fit in frames 1+ (carry only the frame header).
static int payload_capacity_other(int cells_per_frame) {
    const int overhead_bits = static_cast<int>(camcom::FRAME_HEADER_SIZE * 8);
    const int available = cells_per_frame - overhead_bits;
    return (available > 0) ? (available / 8) : 0;
}

/// Minimum number of frames required to carry data_size bytes.
static int required_frames(std::size_t data_size, int pb0, int pb_other) {
    if (data_size == 0) return 1;
    if (static_cast<int>(data_size) <= pb0) return 1;
    const std::size_t remaining = data_size - static_cast<std::size_t>(pb0);
    return 1 + static_cast<int>((remaining + static_cast<std::size_t>(pb_other) - 1)
                                / static_cast<std::size_t>(pb_other));
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    const std::string input_path   = argv[1];
    const std::string output_path  = argv[2];
    const long        duration_ms  = std::stol(argv[3]);

    if (duration_ms <= 0) {
        std::cerr << "Error: duration_ms must be a positive integer.\n";
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    // Verify the input file is readable.
    if (!camcom::file_exists(input_path)) {
        std::cerr << "Error: input file not found: " << input_path << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    std::cout << "[encoder] Input : " << input_path  << "\n"
              << "[encoder] Output: " << output_path << "\n"
              << "[encoder] Duration (ms): " << duration_ms << "\n";

    // Read input binary file.
    const auto data = camcom::read_binary_file(input_path);

    // Encoding parameters.
    const int fps       = camcom::DEFAULT_FPS;
    const int cell_size = camcom::DEFAULT_CELL_SIZE;
    const int width     = camcom::DEFAULT_FRAME_WIDTH;
    const int height    = camcom::DEFAULT_FRAME_HEIGHT;

    const int cells_per_frame = (width / cell_size) * (height / cell_size);
    const int pb0    = payload_capacity_frame0(cells_per_frame);
    const int pb_other = payload_capacity_other(cells_per_frame);

    if (pb0 <= 0 || pb_other <= 0) {
        std::cerr << "Error: cell_size too large — no room for payload in a frame.\n";
        return static_cast<int>(camcom::ExitCode::EncodingError);
    }

    // Total frames derived from the requested duration.
    const int total_frames = static_cast<int>(
        std::ceil(static_cast<double>(duration_ms) * fps / 1000.0));

    // Check that the data fits within the requested duration.
    const int min_frames = required_frames(data.size(),
                                           pb0, pb_other);
    if (min_frames > total_frames) {
        std::cerr << "Error: data (" << data.size() << " bytes) requires at least "
                  << min_frames << " frames but duration_ms=" << duration_ms
                  << " only allows " << total_frames << " frames at "
                  << fps << " fps.\n";
        return static_cast<int>(camcom::ExitCode::EncodingError);
    }

    // Build StreamHeader.
    camcom::StreamHeader stream_hdr{};
    stream_hdr.magic            = camcom::MAGIC;
    stream_hdr.version          = camcom::FORMAT_VERSION;
    stream_hdr.total_data_bytes = static_cast<uint64_t>(data.size());
    stream_hdr.encoding         = camcom::Encoding::Binary;
    stream_hdr.fps              = static_cast<uint32_t>(fps);
    stream_hdr.cell_size        = static_cast<uint32_t>(cell_size);

    const auto shdr_bytes = camcom::serialize_stream_header(stream_hdr);

    // Open VideoWriter (BGR, MPEG-4).
    cv::VideoWriter writer;
    if (!writer.open(output_path, VIDEO_FOURCC, fps,
                     cv::Size(width, height), true)) {
        std::cerr << "Error: cannot open VideoWriter for: " << output_path << "\n";
        return static_cast<int>(camcom::ExitCode::EncodingError);
    }

    // Encode frames.
    std::size_t data_offset = 0;

    for (int fi = 0; fi < total_frames; ++fi) {
        const bool   is_first    = (fi == 0);
        const int    pb_cap      = is_first ? pb0 : pb_other;
        const std::size_t remaining = (data_offset < data.size())
                                     ? (data.size() - data_offset) : 0;
        const std::size_t actual_payload = (remaining < static_cast<std::size_t>(pb_cap))
                                          ? remaining
                                          : static_cast<std::size_t>(pb_cap);

        // Build FrameHeader.
        camcom::FrameHeader frame_hdr{};
        frame_hdr.magic         = camcom::MAGIC;
        frame_hdr.version       = camcom::FORMAT_VERSION;
        frame_hdr.frame_index   = static_cast<uint32_t>(fi);
        frame_hdr.total_frames  = static_cast<uint32_t>(total_frames);
        frame_hdr.payload_bytes = static_cast<uint32_t>(actual_payload);
        frame_hdr.checksum      = (actual_payload > 0)
                                  ? camcom::crc32(data.data() + data_offset, actual_payload)
                                  : 0u;

        const auto fhdr_bytes = camcom::serialize_frame_header(frame_hdr);

        // Assemble the bit vector for this frame.
        std::vector<uint8_t> frame_bits;
        frame_bits.reserve(cells_per_frame);

        if (is_first) {
            const auto sb = bytes_to_bits(shdr_bytes.data(), shdr_bytes.size());
            frame_bits.insert(frame_bits.end(), sb.begin(), sb.end());
        }

        const auto fb = bytes_to_bits(fhdr_bytes.data(), fhdr_bytes.size());
        frame_bits.insert(frame_bits.end(), fb.begin(), fb.end());

        if (actual_payload > 0) {
            const auto pb_vec = bytes_to_bits(data.data() + data_offset, actual_payload);
            frame_bits.insert(frame_bits.end(), pb_vec.begin(), pb_vec.end());
        }

        // Render and write.
        const cv::Mat frame = render_frame(frame_bits, cell_size, width, height);
        writer.write(frame);

        data_offset += actual_payload;
    }

    writer.release();

    std::cout << "[encoder] Encoded " << data.size() << " bytes into "
              << total_frames << " frame(s) -> " << output_path << "\n";
    return static_cast<int>(camcom::ExitCode::Ok);
}
