/**
 * @file common.h
 * @brief Shared types and constants for the CamCom-Binary transmission system.
 *
 * This header is included by both encoder.cpp and decoder.cpp to keep
 * definitions consistent across the project.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Project-wide constants
// ---------------------------------------------------------------------------

/// Default video frame rate (frames per second).
constexpr int DEFAULT_FPS = 30;

/// Default frame width in pixels.
constexpr int DEFAULT_FRAME_WIDTH = 1920;

/// Default frame height in pixels.
constexpr int DEFAULT_FRAME_HEIGHT = 1080;

/// Number of bits encoded per pixel (1 = monochrome black/white).
/// TODO: Increase this value to implement multi-level encoding (e.g. 2 bits/px
///       using gray levels) once you have verified the single-bit baseline.
constexpr int BITS_PER_PIXEL = 1;

/// Pixel value used to represent a logical '1' bit.
constexpr uint8_t PIXEL_ONE = 255;

/// Pixel value used to represent a logical '0' bit.
constexpr uint8_t PIXEL_ZERO = 0;

/// Threshold for decoding: pixels >= this value are treated as '1'.
/// TODO: Tune this threshold based on measured camera response curves to
///       maximise the SNR (Signal-to-Noise Ratio) as per Shannon's theorem.
constexpr uint8_t DECODE_THRESHOLD = 128;

/// Confidence margin for bit validity determination in the decoder.
/// A decoded pixel is considered "valid" when its distance from
/// DECODE_THRESHOLD is at least this many grey-levels.
/// Pixels within this margin are near the noise boundary and are flagged
/// invalid in the validity mask (vout.bin).
/// TODO: Tune this value empirically with your specific camera / screen pair.
constexpr uint8_t CONFIDENCE_MARGIN = 32;

// ---------------------------------------------------------------------------
// Shared data structures
// ---------------------------------------------------------------------------

/**
 * @brief Represents a single video frame as a flat array of pixel bytes.
 *
 * For a DEFAULT_FRAME_WIDTH x DEFAULT_FRAME_HEIGHT grayscale frame the vector
 * contains exactly DEFAULT_FRAME_WIDTH * DEFAULT_FRAME_HEIGHT bytes.
 */
using Frame = std::vector<uint8_t>;

/**
 * @brief Encodes the validity of each decoded bit.
 *
 * Written to the validity-mask file (vout.bin) by the decoder.
 * - 0x01 : bit is considered valid (high confidence).
 * - 0x00 : bit is considered invalid / uncertain (noise, blur, exposure).
 *
 * TODO: Replace the binary valid/invalid flag with a confidence score
 *       (e.g. uint8_t 0–255) once the core pipeline is working.
 */
using ValidityMask = std::vector<uint8_t>;

// ---------------------------------------------------------------------------
// Helper: bit capacity per frame
// ---------------------------------------------------------------------------

/**
 * @brief Returns the maximum number of bits that can be stored in one frame.
 * @param width  Frame width in pixels.
 * @param height Frame height in pixels.
 * @return Bit capacity.
 *
 * This is the Nyquist-limited spatial capacity per frame.
 * Each pixel carries BITS_PER_PIXEL bits, so total capacity =
 *     width * height * BITS_PER_PIXEL.
 *
 * TODO: Apply a guard band / parity region to reserve some pixels for
 *       synchronisation markers and error-correction codes.
 */
inline std::size_t bitsPerFrame(int width = DEFAULT_FRAME_WIDTH,
                                int height = DEFAULT_FRAME_HEIGHT) {
    return static_cast<std::size_t>(width) * static_cast<std::size_t>(height)
           * BITS_PER_PIXEL;
}
