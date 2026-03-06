/**
 * @file decoder.cpp
 * @brief Screen-to-Camera Binary Transmission System — Decoder
 *
 * Usage:
 *   ./decoder <recorded.mp4> <output.bin> <validity_mask.bin>
 *
 *   recorded.mp4      : Video captured by the camera of the encoded screen.
 *   output.bin        : Recovered binary data (best-effort bit stream).
 *   validity_mask.bin : Per-bit validity flags written as a byte stream
 *                       (see vout.bin format in README.md and common.h).
 *
 * Build:
 *   See CMakeLists.txt — requires OpenCV and FFmpeg.
 *
 * Course context:
 *   Computer Networks — Physical Layer project.
 *   The decoder implements the *receiver* side of the channel.
 *   Consider the following real-world impairments when tuning thresholds:
 *     - Motion blur (camera or screen movement).
 *     - Exposure differences (camera auto-exposure changing mid-stream).
 *     - Lens distortion and perspective warp (off-axis camera placement).
 *     - Moiré / aliasing effects between pixel grid and camera sensor.
 *     - Lighting changes (ambient light mixing with screen light).
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>
#include <stdexcept>

// OpenCV headers
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// FFmpeg headers (C interface — must be wrapped in extern "C")
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include "common.h"

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

/**
 * @brief Open a video file and extract all frames as grayscale cv::Mat objects.
 * @param path  Path to the recorded video file.
 * @return      Vector of grayscale frames in display order.
 * @throws std::runtime_error if the file cannot be opened.
 *
 * TODO: Implement this function.
 *   1. Open the video with cv::VideoCapture.
 *   2. Read frames in a loop using cap.read().
 *   3. Convert each frame to grayscale (cv::cvtColor with COLOR_BGR2GRAY).
 *   4. (Optional) Apply geometric correction (cv::warpPerspective) if the
 *      camera is not perfectly aligned with the screen.
 *   5. (Optional) Apply adaptive histogram equalisation (cv::createCLAHE)
 *      to compensate for exposure differences across frames.
 *
 * Noise note: hardware H.264 decoding may introduce block artefacts.
 *   Apply a mild Gaussian blur (cv::GaussianBlur, kernel 3×3, σ=0.5) BEFORE
 *   thresholding to reduce false bit flips caused by codec ringing.
 */
std::vector<cv::Mat> decodeVideoToFrames(const std::string& path);

/**
 * @brief Extract a bit stream from a sequence of decoded video frames.
 * @param frames   Grayscale frames from decodeVideoToFrames().
 * @param width    Expected frame width (must match encoder settings).
 * @param height   Expected frame height (must match encoder settings).
 * @param[out] validityMask  Per-bit validity flags (same length as returned bits).
 * @return         Recovered bit stream packed into bytes (MSB first per byte).
 *
 * TODO: Implement this function.
 *   1. For each pixel in each frame, compare its value to DECODE_THRESHOLD.
 *        pixel >= DECODE_THRESHOLD  →  bit 1
 *        pixel <  DECODE_THRESHOLD  →  bit 0
 *   2. Assign a validity flag:
 *        |pixel - DECODE_THRESHOLD| >= CONFIDENCE_MARGIN  →  valid (0x01)
 *        otherwise                                         →  invalid (0x00)
 *      where CONFIDENCE_MARGIN reflects how far the pixel is from the
 *      threshold; pixels near the boundary are noisy/uncertain.
 *   3. Pack bits into bytes (8 bits per byte, MSB first).
 *   4. (Optional) Detect and skip the synchronisation / preamble frame that
 *      the encoder inserted at index 0 before processing data frames.
 *
 * Shannon note: the bit error rate (BER) is determined by the channel SNR.
 *   Measure the ratio of valid bits from the validity mask to estimate the
 *   effective channel capacity and compare it with the theoretical maximum
 *   given by Shannon's formula: C = B * log2(1 + S/N).
 */
std::vector<uint8_t> extractBitsFromFrames(const std::vector<cv::Mat>& frames,
                                           int width,
                                           int height,
                                           ValidityMask& validityMask);

/**
 * @brief Write a byte vector to a binary file.
 * @param data  Bytes to write.
 * @param path  Destination file path.
 * @throws std::runtime_error if the file cannot be written.
 *
 * TODO: Implement this function.
 *   1. Open the file in binary write mode.
 *   2. Write data.data() for data.size() bytes.
 */
void writeBinaryFile(const std::vector<uint8_t>& data, const std::string& path);

/**
 * @brief Write the validity mask to a binary file (the "vout.bin" file).
 * @param mask  Per-bit validity flags (0x01 = valid, 0x00 = invalid).
 * @param path  Destination file path.
 * @throws std::runtime_error if the file cannot be written.
 *
 * TODO: Implement this function using the same approach as writeBinaryFile().
 *
 * Format description (vout.bin):
 *   - One byte per decoded bit.
 *   - Byte value 0x01: the corresponding output bit is considered reliable.
 *   - Byte value 0x00: the corresponding output bit is uncertain/noisy.
 *   - The i-th byte in vout.bin corresponds to the i-th bit in output.bin
 *     (bit order: MSB of byte 0 first, then LSB of byte 0, then MSB of byte 1 …).
 *
 * Extended usage: a downstream error-correction layer (e.g. Reed-Solomon)
 *   can treat 0x00 positions as *erasures*, which are cheaper to correct
 *   than random errors and allow higher code rates.
 */
void writeValidityMask(const ValidityMask& mask, const std::string& path);

// ---------------------------------------------------------------------------
// Stub implementations
// ---------------------------------------------------------------------------

std::vector<cv::Mat> decodeVideoToFrames(const std::string& path) {
    // TODO: Implement full video decoding (see doc above).
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        throw std::runtime_error("Cannot open video file: " + path);
    }

    std::vector<cv::Mat> frames;
    cv::Mat frameColor;
    while (cap.read(frameColor)) {
        cv::Mat gray;
        cv::cvtColor(frameColor, gray, cv::COLOR_BGR2GRAY);
        // TODO: Apply perspective correction here if the camera is off-axis.
        // TODO: Apply exposure normalisation (CLAHE) here if needed.
        frames.push_back(std::move(gray));
    }
    cap.release();

    std::cout << "[decoder] Decoded " << frames.size() << " frame(s) from "
              << path << "\n";
    return frames;
}

std::vector<uint8_t> extractBitsFromFrames(const std::vector<cv::Mat>& frames,
                                           int width,
                                           int height,
                                           ValidityMask& validityMask) {
    // TODO: Implement full bit extraction (see doc above).
    //       The stub applies a simple threshold to each pixel.

    std::vector<bool> bits;
    validityMask.clear();

    for (const auto& frame : frames) {
        for (int row = 0; row < height && row < frame.rows; ++row) {
            for (int col = 0; col < width && col < frame.cols; ++col) {
                uint8_t pixel = frame.at<uint8_t>(row, col);
                bool    bit   = (pixel >= DECODE_THRESHOLD);
                bits.push_back(bit);

                // Validity: distance from threshold must exceed margin.
                int distance = std::abs(static_cast<int>(pixel)
                                        - static_cast<int>(DECODE_THRESHOLD));
                validityMask.push_back(
                    (distance >= CONFIDENCE_MARGIN) ? 0x01 : 0x00);
            }
        }
    }

    // Pack bits into bytes (MSB first)
    std::vector<uint8_t> packed;
    packed.reserve((bits.size() + 7) / 8);
    for (std::size_t i = 0; i < bits.size(); i += 8) {
        uint8_t byte = 0;
        for (int b = 0; b < 8 && (i + b) < bits.size(); ++b) {
            if (bits[i + b]) {
                byte |= static_cast<uint8_t>(0x80u >> b);
            }
        }
        packed.push_back(byte);
    }
    return packed;
}

void writeBinaryFile(const std::vector<uint8_t>& data, const std::string& path) {
    // TODO: Implement (see doc above).
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open output file for writing: " + path);
    }
    file.write(reinterpret_cast<const char*>(data.data()),
               static_cast<std::streamsize>(data.size()));
    if (!file) {
        throw std::runtime_error("Failed to write to file: " + path);
    }
}

void writeValidityMask(const ValidityMask& mask, const std::string& path) {
    // TODO: Implement (see doc above).
    writeBinaryFile(mask, path);
}

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // ------------------------------------------------------------------
    // Argument parsing
    // ------------------------------------------------------------------
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <recorded.mp4> <output.bin> <validity_mask.bin>\n"
                  << "\n"
                  << "  recorded.mp4      : Camera-recorded video of the encoded screen.\n"
                  << "  output.bin        : Recovered binary data.\n"
                  << "  validity_mask.bin : Per-bit validity flags (vout.bin format).\n";
        return EXIT_FAILURE;
    }

    const std::string videoPath    = argv[1];
    const std::string outputPath   = argv[2];
    const std::string maskPath     = argv[3];

    std::cout << "[decoder] Input video  : " << videoPath  << "\n"
              << "[decoder] Output data  : " << outputPath << "\n"
              << "[decoder] Validity mask: " << maskPath   << "\n";

    // ------------------------------------------------------------------
    // Pipeline
    // ------------------------------------------------------------------
    try {
        // Step 1: Open video and extract grayscale frames
        std::cout << "[decoder] Decoding video to frames...\n";
        auto frames = decodeVideoToFrames(videoPath);

        // Step 2: Extract bits and build validity mask
        // TODO: Pass the expected frame dimensions from a config file or
        //       embedded metadata so that the decoder does not need to
        //       hard-code the resolution.
        std::cout << "[decoder] Extracting bits from frames...\n";
        ValidityMask mask;
        auto data = extractBitsFromFrames(frames,
                                          DEFAULT_FRAME_WIDTH,
                                          DEFAULT_FRAME_HEIGHT,
                                          mask);

        // Log validity statistics for debugging / course report
        std::size_t validCount = 0;
        for (auto v : mask) {
            if (v == 0x01) ++validCount;
        }
        double validRatio = mask.empty() ? 0.0
                          : (static_cast<double>(validCount) / mask.size()) * 100.0;
        std::cout << "[decoder] Recovered " << data.size() << " byte(s), "
                  << mask.size() << " bits total.\n"
                  << "[decoder] Valid bits: " << validCount
                  << " / " << mask.size()
                  << " (" << validRatio << "%).\n";

        // Step 3: Write recovered data
        std::cout << "[decoder] Writing output binary...\n";
        writeBinaryFile(data, outputPath);

        // Step 4: Write validity mask
        std::cout << "[decoder] Writing validity mask...\n";
        writeValidityMask(mask, maskPath);

    } catch (const std::exception& e) {
        std::cerr << "[decoder] ERROR: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "[decoder] Done.\n";
    return EXIT_SUCCESS;
}
