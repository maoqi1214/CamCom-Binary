#include "common.hpp"
#include "io.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

/// Pixel intensity threshold: pixels strictly above this value are decoded as
/// bit 1 (white cell); at or below as bit 0 (black cell).
static constexpr uint8_t PIXEL_THRESHOLD = 128;

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
// Bit-level helpers
// ---------------------------------------------------------------------------

/// Sample a grayscale frame: for each cell, read the centre pixel and
/// threshold at 128 to produce one bit (0 or 1).
static std::vector<uint8_t> sample_frame(const cv::Mat& gray,
                                          int cell_size) {
    const int cols = gray.cols / cell_size;
    const int rows = gray.rows / cell_size;
    std::vector<uint8_t> bits;
    bits.reserve(static_cast<std::size_t>(cols * rows));
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const int px = c * cell_size + cell_size / 2;
            const int py = r * cell_size + cell_size / 2;
            const uint8_t pixel = gray.at<uint8_t>(py, px);
            bits.push_back(pixel > PIXEL_THRESHOLD ? 1u : 0u);
        }
    }
    return bits;
}

/// Convert a bit vector (MSB first per byte) to a byte vector.
static std::vector<uint8_t> bits_to_bytes(const std::vector<uint8_t>& bits) {
    std::vector<uint8_t> bytes;
    bytes.reserve(bits.size() / 8);
    const std::size_t n = (bits.size() / 8) * 8; // round down to byte boundary
    for (std::size_t i = 0; i < n; i += 8) {
        uint8_t b = 0;
        for (int j = 0; j < 8; ++j)
            b = static_cast<uint8_t>((b << 1) | (bits[i + j] & 1u));
        bytes.push_back(b);
    }
    return bytes;
}

// ---------------------------------------------------------------------------
// Offset calculation
// ---------------------------------------------------------------------------

/// Compute the byte offset in the reconstructed data buffer where frame
/// frame_index's payload begins.
static std::size_t frame_data_offset(uint32_t frame_index,
                                      int cells_per_frame) {
    const int pb0 = (cells_per_frame
                     - static_cast<int>((camcom::STREAM_HEADER_SIZE
                                         + camcom::FRAME_HEADER_SIZE) * 8)) / 8;
    const int pb_other = (cells_per_frame
                          - static_cast<int>(camcom::FRAME_HEADER_SIZE * 8)) / 8;

    if (frame_index == 0)
        return 0;

    return static_cast<std::size_t>(pb0)
         + static_cast<std::size_t>(frame_index - 1) * static_cast<std::size_t>(pb_other);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(camcom::ExitCode::BadArgs);
    }

    const std::string video_path        = argv[1];
    const std::string output_path       = argv[2];
    const std::string mask_path         = argv[3];

    // Verify the input video file is readable.
    if (!camcom::file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    std::cout << "[decoder] Video : " << video_path  << "\n"
              << "[decoder] Output: " << output_path << "\n"
              << "[decoder] Mask  : " << mask_path   << "\n";

    // Open the video file.
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: cannot open video: " << video_path << "\n";
        return static_cast<int>(camcom::ExitCode::IoError);
    }

    const int vid_width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int vid_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    // Use default cell size to bootstrap decoding of frame 0.
    int cell_size = camcom::DEFAULT_CELL_SIZE;

    std::vector<uint8_t> recovered_data;
    std::vector<uint8_t> validity_mask;
    bool stream_hdr_read   = false;
    uint32_t total_frames  = 0;

    cv::Mat raw_frame;
    int fi = 0;

    while (cap.read(raw_frame)) {
        // Convert to grayscale.
        cv::Mat gray;
        if (raw_frame.channels() == 1)
            gray = raw_frame;
        else
            cv::cvtColor(raw_frame, gray, cv::COLOR_BGR2GRAY);

        // If we got the cell_size from StreamHeader and it differs from default,
        // we may need to re-check the dimensions (defensive: use actual video dims).
        const int cols = vid_width  / cell_size;
        const int rows = vid_height / cell_size;
        const int cells_per_frame = cols * rows;

        // Sample the frame into a bit vector then convert to bytes.
        const std::vector<uint8_t> bits  = sample_frame(gray, cell_size);
        const std::vector<uint8_t> bytes = bits_to_bytes(bits);

        std::size_t offset = 0;

        // --- Frame 0: read StreamHeader first ---
        if (fi == 0) {
            camcom::StreamHeader shdr{};
            if (!camcom::deserialize_stream_header(bytes.data(), bytes.size(), shdr)) {
                std::cerr << "Error: frame 0 too small to contain StreamHeader.\n";
                return static_cast<int>(camcom::ExitCode::DecodingError);
            }
            if (shdr.magic != camcom::MAGIC) {
                std::cerr << "Error: StreamHeader magic mismatch in frame 0.\n";
                return static_cast<int>(camcom::ExitCode::DecodingError);
            }
            if (shdr.version != camcom::FORMAT_VERSION) {
                std::cerr << "Error: unsupported format version: "
                          << static_cast<int>(shdr.version) << "\n";
                return static_cast<int>(camcom::ExitCode::DecodingError);
            }

            // Update cell_size from stream header.
            cell_size = static_cast<int>(shdr.cell_size);
            stream_hdr_read = true;

            recovered_data.assign(static_cast<std::size_t>(shdr.total_data_bytes), 0);
            validity_mask.assign(static_cast<std::size_t>(shdr.total_data_bytes), 0);

            offset = camcom::STREAM_HEADER_SIZE;
        }

        if (!stream_hdr_read) {
            std::cerr << "Error: StreamHeader not yet read at frame " << fi << ".\n";
            return static_cast<int>(camcom::ExitCode::DecodingError);
        }

        // --- Read FrameHeader ---
        camcom::FrameHeader fhdr{};
        if (!camcom::deserialize_frame_header(bytes.data() + offset,
                                              bytes.size() - offset, fhdr)) {
            std::cerr << "Warning: frame " << fi << " too small for FrameHeader, skipping.\n";
            ++fi;
            continue;
        }

        if (fhdr.magic != camcom::MAGIC) {
            std::cerr << "Warning: FrameHeader magic mismatch at frame " << fi
                      << ", skipping.\n";
            ++fi;
            continue;
        }

        if (fi == 0)
            total_frames = fhdr.total_frames;

        offset += camcom::FRAME_HEADER_SIZE;

        // --- Read payload ---
        const uint32_t payload_bytes = fhdr.payload_bytes;

        if (payload_bytes > 0 && offset + payload_bytes <= bytes.size()) {
            const uint8_t* payload_ptr = bytes.data() + offset;

            // Validate CRC-32.
            const uint32_t actual_crc = camcom::crc32(payload_ptr, payload_bytes);
            const bool valid = (actual_crc == fhdr.checksum);

            // Write payload into recovered buffer at the correct data offset.
            const std::size_t data_start =
                frame_data_offset(fhdr.frame_index, cells_per_frame);

            for (uint32_t i = 0; i < payload_bytes; ++i) {
                const std::size_t pos = data_start + i;
                if (pos < recovered_data.size()) {
                    recovered_data[pos]  = payload_ptr[i];
                    validity_mask[pos]   = valid ? 1u : 0u;
                }
            }
        }

        ++fi;

        // Stop once we have read all declared frames.
        if (total_frames > 0 && static_cast<uint32_t>(fi) >= total_frames)
            break;
    }

    cap.release();

    if (!stream_hdr_read || fi == 0) {
        std::cerr << "Error: no valid frames decoded from " << video_path << "\n";
        return static_cast<int>(camcom::ExitCode::DecodingError);
    }

    // Write recovered bytes.
    camcom::write_binary_file(output_path, recovered_data);

    // Write per-byte validity mask.
    camcom::write_binary_file(mask_path, validity_mask);

    // Count valid bytes.
    std::size_t valid_count = 0;
    for (uint8_t m : validity_mask) valid_count += m;

    std::cout << "[decoder] Decoded " << recovered_data.size() << " byte(s) ("
              << valid_count << " valid) -> " << output_path << "\n"
              << "[decoder] Validity mask -> " << mask_path << "\n";

    return static_cast<int>(camcom::ExitCode::Ok);
}
