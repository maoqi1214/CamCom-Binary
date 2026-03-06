# CamCom-Binary

Screen-to-Camera Binary Transmission System written in C++17.

CamCom-Binary encodes an arbitrary binary file into a video that can be played
on a screen, recorded with a camera, and then decoded back to the original
bytes — along with a per-byte validity mask indicating decoding confidence.

> **Status:** Initial skeleton.  Encoding / decoding logic is marked with
> `TODO` placeholders and not yet implemented.

---

## Repository Layout

```
CamCom-Binary/
├── CMakeLists.txt       # Build configuration (CMake ≥ 3.16)
├── include/
│   ├── common.hpp       # Shared constants, types, structs, enums
│   └── io.hpp           # I/O helper declarations
├── src/
│   ├── common.cpp       # CRC-32 and other shared utilities
│   ├── io.cpp           # Binary file read/write helpers
│   ├── encoder.cpp      # Encoder entry point (TODO)
│   └── decoder.cpp      # Decoder entry point (TODO)
├── tests/
│   ├── README.md        # How to run smoke tests
│   └── sample_input.bin # 16-byte fixture for smoke testing
├── docs/
│   ├── format.md        # Binary / video format specification
│   ├── usage.md         # Build and CLI usage guide
│   └── design.md        # High-level design notes
├── LICENSE
└── README.md            # This file
```

---

## Dependencies

| Library | Purpose |
|---------|---------|
| [OpenCV](https://opencv.org/) | Frame rendering and image processing |
| [FFmpeg](https://ffmpeg.org/) (`libavcodec`, `libavformat`, `libavutil`) | Video encode / decode |

### Linux (Ubuntu / Debian)

```bash
sudo apt install cmake \
    libopencv-dev \
    ffmpeg libavcodec-dev libavformat-dev libavutil-dev
```

### macOS (Homebrew)

```bash
brew install cmake opencv ffmpeg
```

### Windows

Install dependencies via [vcpkg](https://github.com/microsoft/vcpkg):

```powershell
vcpkg install opencv ffmpeg
```

---

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)   # Linux / macOS
# or: cmake --build . --config Release   # Windows
```

Binaries are placed in `build/bin/`.

---

## Encoder CLI

```
encoder <input.bin> <output.mp4> <duration_ms>
```

| Argument      | Description                                 |
|---------------|---------------------------------------------|
| `input.bin`   | Binary file to encode                       |
| `output.mp4`  | Output video file                           |
| `duration_ms` | Total video duration in milliseconds        |

### Example

```bash
build/bin/encoder payload.bin out.mp4 5000
```

---

## Decoder CLI

```
decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

| Argument            | Description                                    |
|---------------------|------------------------------------------------|
| `recorded.mp4`      | Video captured by a camera                     |
| `output.bin`        | Path to write the recovered binary data        |
| `validity_mask.bin` | Path to write the per-byte validity mask       |

### Example

```bash
build/bin/decoder captured.mp4 recovered.bin mask.bin
```

---

## Validity Mask Format

`validity_mask.bin` contains one byte per byte of recovered data.
`0x01` = byte recovered with confidence; `0x00` = byte undetermined.

See [`docs/format.md`](docs/format.md) for the full file format specification.

---

## Notes

- All encoding / decoding logic is currently stubbed with `TODO` markers.
- The file format version is `1` (field `FORMAT_VERSION` in `include/common.hpp`).
- See [`docs/design.md`](docs/design.md) for architecture notes and
  [`docs/usage.md`](docs/usage.md) for a more detailed usage guide.
