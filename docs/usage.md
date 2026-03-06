# Usage Guide

> **Status:** Placeholder — to be updated as the implementation matures.

## Building

```bash
# Install dependencies (Ubuntu/Debian example)
sudo apt install cmake libopencv-dev ffmpeg libavcodec-dev libavformat-dev libavutil-dev

# Configure and build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Binaries are placed in `build/bin/`.

## Encoder

```
encoder <input.bin> <output.mp4> <duration_ms>
```

| Argument      | Description                                   |
|---------------|-----------------------------------------------|
| `input.bin`   | Binary file to encode                         |
| `output.mp4`  | Output video file                             |
| `duration_ms` | Total video duration in milliseconds          |

### Example

```bash
build/bin/encoder payload.bin out.mp4 5000
```

## Decoder

```
decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

| Argument            | Description                                       |
|---------------------|---------------------------------------------------|
| `recorded.mp4`      | Video captured by a camera                        |
| `output.bin`        | Path to write the recovered binary data           |
| `validity_mask.bin` | Path to write the per-bit validity mask           |

### Example

```bash
build/bin/decoder captured.mp4 recovered.bin mask.bin
```

## Validity Mask

After decoding, inspect `mask.bin` to see which bytes were recovered
successfully. See `docs/format.md` for the mask format.

## TODO

- Document supported encoding modes.
- Document camera setup recommendations (resolution, frame-rate, lighting).
- Provide end-to-end worked example.
