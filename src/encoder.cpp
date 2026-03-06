/**
 * @file encoder.cpp
 * @brief Screen-to-Camera Binary Transmission System — Encoder
 *
 * Usage:
 *   ./encoder <input.bin> <output.mp4> <duration_ms>
 *
 *   input.bin    : Binary file to transmit (arbitrary bytes).
 *   output.mp4   : Output video file that will be displayed on screen and
 *                  recorded by a camera.
 *   duration_ms  : Total transmission duration in milliseconds.
 *                  The encoder will spread the data across the calculated
 *                  number of video frames.
 *
 * Build:
 *   See CMakeLists.txt — requires OpenCV and FFmpeg.
 *
 * Course context:
 *   Computer Networks — Physical Layer project.
 *   The encoder implements the *transmitter* side of the channel.
 *   Consult Shannon's channel-capacity theorem and the Nyquist sampling
 *   theorem when choosing frame rate, resolution, and bits-per-pixel.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdint>
#include <stdexcept>
#include <cmath>

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
 * @brief Read the entire contents of a binary file into a byte vector.
 * @param path  Path to the input binary file.
 * @return      Vector of bytes from the file.
 * @throws std::runtime_error if the file cannot be opened.
 *
 * TODO: Implement this function.
 *   1. Open the file in binary mode (std::ios::binary).
 *   2. Determine file size using seekg/tellg or std::filesystem::file_size.
 *   3. Read all bytes into the returned vector.
 */
std::vector<uint8_t> readBinaryFile(const std::string& path);

/**
 * @brief Convert a byte vector into a sequence of video frames.
 * @param data      Raw bytes to encode.
 * @param frameRate Frames per second of the output video.
 * @param width     Frame width in pixels.
 * @param height    Frame height in pixels.
 * @return          Vector of OpenCV Mat objects (grayscale frames).
 *
 * TODO: Implement this function.
 *   1. Compute total bit count from data.size() * 8.
 *   2. Apply Nyquist / Shannon logic:
 *        - Each frame can hold bitsPerFrame(width, height) bits.
 *        - Minimum frames needed = ceil(totalBits / bitsPerFrame).
 *   3. For each bit, map it to a pixel:
 *        - bit == 1  →  PIXEL_ONE  (white)
 *        - bit == 0  →  PIXEL_ZERO (black)
 *   4. Fill remaining pixels in the last frame with PIXEL_ZERO.
 *   5. Optionally insert a known synchronisation / preamble frame at index 0
 *      so the decoder can align itself temporally.
 *
 * Nyquist note: the spatial sampling rate is 1 sample per pixel. For
 * reliable decoding the minimum feature size should be at least 2 pixels
 * per bit (Nyquist criterion) to survive camera blur. Consider grouping
 * bits into NxN pixel blocks (e.g. 4×4) for robustness.
 */
std::vector<cv::Mat> encodeBitsToFrames(const std::vector<uint8_t>& data,
                                        int frameRate,
                                        int width,
                                        int height);

/**
 * @brief Write a sequence of frames to a video file using OpenCV / FFmpeg.
 * @param frames     Frames to write (grayscale cv::Mat).
 * @param outputPath Destination .mp4 path.
 * @param frameRate  Frames per second.
 * @param width      Frame width in pixels.
 * @param height     Frame height in pixels.
 * @throws std::runtime_error if the video writer cannot be opened.
 *
 * TODO: Implement this function.
 *   1. Open a cv::VideoWriter with the chosen codec (e.g. H.264 / MJPEG).
 *   2. For each frame in `frames`, write it to the video.
 *   3. Release the writer when done.
 *   4. (Advanced) Use FFmpeg's API directly (avcodec_send_frame /
 *      avcodec_receive_packet) if you need finer control over codec
 *      parameters (bitrate, GOP size, etc.) to minimise compression
 *      artefacts that distort the encoded bits.
 *
 * Compression warning: lossy codecs (H.264 default settings) will alter
 * pixel values. Either use a lossless preset ("yuv444p" + "lossless") or
 * rely on the threshold-based decoder to recover bits despite minor changes.
 */
void generateVideo(const std::vector<cv::Mat>& frames,
                   const std::string& outputPath,
                   int frameRate,
                   int width,
                   int height);

// ---------------------------------------------------------------------------
// Stub implementations
// ---------------------------------------------------------------------------

std::vector<uint8_t> readBinaryFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open input file: " + path);
    }
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<uint8_t> buffer(static_cast<std::size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        throw std::runtime_error("Failed to read input file: " + path);
    }
    return buffer;
}

std::vector<cv::Mat> encodeBitsToFrames(const std::vector<uint8_t>& data,
                                        int frameRate,
                                        int width,
                                        int height) {
    (void)frameRate; // will be used when timing logic is added

    // TODO: Implement full encoding logic (see doc above).
    //       The stub below creates one placeholder black frame per byte
    //       so that the build succeeds and the pipeline can be tested end-to-end.

    std::vector<cv::Mat> frames;

    std::size_t totalBits = data.size() * 8;
    std::size_t capacity  = bitsPerFrame(width, height);
    std::size_t numFrames = (totalBits + capacity - 1) / capacity;
    if (numFrames == 0) numFrames = 1;

    frames.reserve(numFrames);

    std::size_t bitIndex = 0;
    for (std::size_t f = 0; f < numFrames; ++f) {
        cv::Mat frame(height, width, CV_8UC1, cv::Scalar(PIXEL_ZERO));
        for (int row = 0; row < height && bitIndex < totalBits; ++row) {
            for (int col = 0; col < width && bitIndex < totalBits; ++col) {
                // TODO: Replace flat pixel mapping with an NxN block mapping
                //       to improve robustness against camera blur.
                uint8_t byte = data[bitIndex / 8];
                int     bit  = (byte >> (7 - (bitIndex % 8))) & 0x01;
                frame.at<uint8_t>(row, col) = (bit == 1) ? PIXEL_ONE : PIXEL_ZERO;
                ++bitIndex;
            }
        }
        frames.push_back(std::move(frame));
    }
    return frames;
}

void generateVideo(const std::vector<cv::Mat>& frames,
                   const std::string& outputPath,
                   int frameRate,
                   int width,
                   int height) {
    // TODO: Implement video writing (see doc above).
    //       The stub uses OpenCV's built-in VideoWriter with MJPEG codec.
    //       For a lossless output, switch the fourcc to
    //           cv::VideoWriter::fourcc('F','F','V','1')
    //       and ensure the container supports it (e.g. .avi or .mkv).

    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::VideoWriter writer(outputPath, fourcc, frameRate,
                           cv::Size(width, height), false /* isColor=false */);
    if (!writer.isOpened()) {
        throw std::runtime_error("Cannot open VideoWriter for: " + outputPath);
    }

    for (const auto& frame : frames) {
        writer.write(frame);
    }
    writer.release();

    std::cout << "[encoder] Video written to: " << outputPath
              << " (" << frames.size() << " frames @ " << frameRate << " fps)\n";
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
                  << " <input.bin> <output.mp4> <duration_ms>\n"
                  << "\n"
                  << "  input.bin    : Binary file to encode into video frames.\n"
                  << "  output.mp4   : Output video file path.\n"
                  << "  duration_ms  : Total desired transmission duration (ms).\n";
        return EXIT_FAILURE;
    }

    const std::string inputPath  = argv[1];
    const std::string outputPath = argv[2];
    int durationMs = 0;
    try {
        durationMs = std::stoi(argv[3]);
    } catch (const std::exception& e) {
        std::cerr << "[encoder] Invalid duration_ms: " << argv[3] << "\n";
        return EXIT_FAILURE;
    }
    if (durationMs <= 0) {
        std::cerr << "[encoder] duration_ms must be a positive integer.\n";
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // Compute frame rate from requested duration
    // ------------------------------------------------------------------
    // TODO: Refine this calculation using Shannon's theorem.
    //   Channel capacity C = B * log2(1 + S/N)
    //   where B is the bandwidth (fps * pixels_per_frame) and S/N is the
    //   signal-to-noise ratio of the camera channel.
    //   For now we use DEFAULT_FPS as a fixed frame rate.
    const int frameRate = DEFAULT_FPS;
    const int totalFrames = static_cast<int>(
        std::ceil(static_cast<double>(durationMs) / 1000.0 * frameRate));

    std::cout << "[encoder] Input  : " << inputPath  << "\n"
              << "[encoder] Output : " << outputPath << "\n"
              << "[encoder] FPS    : " << frameRate  << "\n"
              << "[encoder] Total frames (from duration): " << totalFrames << "\n";

    // ------------------------------------------------------------------
    // Pipeline
    // ------------------------------------------------------------------
    try {
        // Step 1: Read input binary file
        std::cout << "[encoder] Reading binary file...\n";
        auto data = readBinaryFile(inputPath);
        std::cout << "[encoder] Read " << data.size() << " bytes ("
                  << data.size() * 8 << " bits).\n";

        // Step 2: Encode bits to frames
        // TODO: Pass totalFrames as a constraint so the encoder can pad/trim
        //       the frame sequence to exactly match the requested duration.
        std::cout << "[encoder] Encoding bits to frames...\n";
        auto frames = encodeBitsToFrames(data, frameRate,
                                         DEFAULT_FRAME_WIDTH,
                                         DEFAULT_FRAME_HEIGHT);
        std::cout << "[encoder] Generated " << frames.size() << " frame(s).\n";

        // Step 3: Write video
        std::cout << "[encoder] Writing video...\n";
        generateVideo(frames, outputPath, frameRate,
                      DEFAULT_FRAME_WIDTH, DEFAULT_FRAME_HEIGHT);

    } catch (const std::exception& e) {
        std::cerr << "[encoder] ERROR: " << e.what() << "\n";
        return EXIT_FAILURE;
    }

    std::cout << "[encoder] Done.\n";
    return EXIT_SUCCESS;
}
