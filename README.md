# CamCom-Binary

**Screen-to-Camera Binary Transmission System**  
*Computer Networks Course Project — Physical Layer*

---

## Project Goals

This project implements a **visible-light communication channel** that transmits
arbitrary binary data by:

1. **Encoding** a binary file as a sequence of black-and-white video frames
   displayed on a computer screen (the *transmitter*).
2. **Decoding** a camera recording of that screen back into the original binary
   file (the *receiver*).

### Physical-Layer Concepts

| Concept | Application in this project |
|---------|-----------------------------|
| **Nyquist Sampling Theorem** | Minimum pixel block size so that camera blur does not destroy bit boundaries. Each bit must span ≥ 2 pixels (Nyquist criterion). |
| **Shannon Channel Capacity** | `C = B · log₂(1 + S/N)` bounds the maximum reliable bit rate given the camera's SNR. Used to select frame rate and bits-per-pixel. |
| **Bit Error Rate (BER)** | Measured via the validity mask (`vout.bin`). Invalid bits represent the noise floor of the channel. |

---

## Directory Structure

```
CamCom-Binary/
├── CMakeLists.txt        # Build configuration (OpenCV + FFmpeg)
├── README.md             # This file
├── .gitignore
├── include/
│   └── common.h          # Shared constants, types, and helpers
├── src/
│   ├── encoder.cpp       # Transmitter: binary file → video
│   └── decoder.cpp       # Receiver:   video → binary file + validity mask
├── tests/
│   ├── README.md         # Test workflow instructions
│   └── sample_input.bin  # 32-byte test pattern (0x00–0x1F)
├── build/                # CMake build output (git-ignored)
└── docs/                 # Project reports and writeups
```

---

## Dependencies

### OpenCV (≥ 4.x)

Used for reading/writing video frames and applying image processing.

<details>
<summary>Windows</summary>

1. Download the pre-built installer from https://opencv.org/releases/
2. Add `C:\opencv\build\x64\vc16\bin` to your `PATH`.
3. Set the `OpenCV_DIR` CMake variable:
   ```
   cmake -DOpenCV_DIR=C:/opencv/build ..
   ```

</details>

<details>
<summary>Linux (Debian / Ubuntu)</summary>

```bash
sudo apt update
sudo apt install libopencv-dev
```

</details>

<details>
<summary>macOS (Homebrew)</summary>

```bash
brew install opencv
```

</details>

---

### FFmpeg (libavcodec, libavformat, libavutil)

Used for low-level video codec access when fine-grained control over
compression parameters is needed.

<details>
<summary>Windows</summary>

1. Download a shared build from https://github.com/BtbN/FFmpeg-Builds/releases
2. Extract to e.g. `C:\ffmpeg`.
3. Add `C:\ffmpeg\bin` to `PATH`.
4. Set `PKG_CONFIG_PATH` to `C:\ffmpeg\lib\pkgconfig` (requires pkg-config for
   Windows: https://www.freedesktop.org/wiki/Software/pkg-config/).

</details>

<details>
<summary>Linux (Debian / Ubuntu)</summary>

```bash
sudo apt update
sudo apt install libavcodec-dev libavformat-dev libavutil-dev pkg-config
```

</details>

<details>
<summary>macOS (Homebrew)</summary>

```bash
brew install ffmpeg pkg-config
```

</details>

---

## Building with CMake

```bash
# 1. Create and enter the build directory
mkdir build && cd build

# 2. Configure (add -DOpenCV_DIR=... if OpenCV is not found automatically)
cmake ..

# 3. Compile
cmake --build . --config Release

# Executables will be placed in build/bin/
#   build/bin/encoder
#   build/bin/decoder
```

---

## Usage

### Encoder

```
./build/bin/encoder <input.bin> <output.mp4> <duration_ms>
```

| Argument | Description |
|----------|-------------|
| `input.bin` | Binary file to transmit. |
| `output.mp4` | Output video to display on screen and record with a camera. |
| `duration_ms` | Desired transmission duration in milliseconds. |

**Example** — encode the provided test file with a 2-second transmission window:

```bash
./build/bin/encoder tests/sample_input.bin /tmp/encoded.mp4 2000
```

---

### Decoder

```
./build/bin/decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

| Argument | Description |
|----------|-------------|
| `recorded.mp4` | Camera recording of the screen showing the encoded video. |
| `output.bin` | Recovered binary data. |
| `validity_mask.bin` | Per-bit validity flags (`vout.bin` format — see below). |

**Example** — decode a camera recording:

```bash
./build/bin/decoder /tmp/recorded.mp4 /tmp/decoded.bin /tmp/vout.bin
```

**Verify round-trip** (software loop-back, no camera):

```bash
./build/bin/decoder /tmp/encoded.mp4 /tmp/decoded.bin /tmp/vout.bin
cmp tests/sample_input.bin /tmp/decoded.bin && echo "PASS" || echo "MISMATCH"
```

---

## `vout.bin` — Validity Mask Format

The validity mask written by the decoder (`<validity_mask.bin>`) has exactly
one byte per decoded bit:

| Byte value | Meaning |
|------------|---------|
| `0x01` | The corresponding bit is **valid** (pixel intensity was far from the decode threshold — high confidence). |
| `0x00` | The corresponding bit is **invalid / uncertain** (pixel was close to the threshold — noisy, blurred, or over/under-exposed). |

**Bit ordering:** the *i*-th byte in `vout.bin` corresponds to the *i*-th bit in
`output.bin`, where bits within a byte are ordered MSB first (bit 7 of byte 0 →
index 0, bit 6 of byte 0 → index 1, …, bit 0 of byte 0 → index 7, bit 7 of byte
1 → index 8, etc.).

**Use case:** a downstream error-correction layer (e.g. Reed-Solomon code) can
treat `0x00` positions as *erasures*, which are cheaper to correct than unknown
random errors and allow higher effective code rates.

---

## Course Requirements Checklist

- [ ] Implement `readBinaryFile()` in `src/encoder.cpp`
- [ ] Implement `encodeBitsToFrames()` — apply Nyquist block mapping (NxN pixels per bit)
- [ ] Implement `generateVideo()` — choose lossless or near-lossless codec
- [ ] Implement `decodeVideoToFrames()` — add perspective correction if needed
- [ ] Implement `extractBitsFromFrames()` — tune `DECODE_THRESHOLD` and `CONFIDENCE_MARGIN`
- [ ] Implement `writeBinaryFile()` and `writeValidityMask()`
- [ ] Measure BER and compare with Shannon's theoretical capacity
- [ ] Write project report in `docs/`

---

## License

See [LICENSE](LICENSE).
