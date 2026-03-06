#include "common.hpp"
#include "io.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

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

    // TODO: Open the video file with FFmpeg and decode each frame.

    // TODO: For each decoded frame, locate and sample the visual data cells.

    // TODO: Convert sampled cells back to bytes, building the payload buffer.

    // TODO: Validate frame headers (magic, version, checksum).

    // TODO: Write recovered bytes to output_path.
    // camcom::write_binary_file(output_path, recovered_data);

    // TODO: Write the per-bit validity mask to mask_path.
    // camcom::write_binary_file(mask_path, validity_mask);

    std::cout << "[decoder] TODO: decoding not yet implemented.\n";
    return static_cast<int>(camcom::ExitCode::Ok);
}
