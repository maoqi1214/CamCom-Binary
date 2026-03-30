# CamCom-Binary

`CamCom-Binary` is a C++/OpenCV/FFmpeg experiment for sending binary data through a rendered color grid video and recovering it from recorded footage.

The repository currently provides two command-line tools:

- `encode`: convert a binary file into a video
- `decode`: recover a binary file from a video

## Current Status

The current codebase is tuned for short, high-throughput clips and phone-recorded decoding experiments.

Current default pipeline:

- fixed encoder frame rate: `20 FPS`
- default `cell_size`: `10`
- default `cells_per_row`: `219`
- encoded payload capacity: `7423 bytes/frame`
- FEC default: `RS(255,239)` with interleave depth `4`
- bootstrap + stream header are multiplexed into the first data frame
- there are no standalone non-data frames in the current encoder path
- the frame now uses a mask-based data layout with a small quiet-zone border
- finder markers have a reserved safety margin around them
- reference color blocks are placed inside the top quiet zone instead of the payload area

With the current defaults, a `1000 ms` clip typically carries about `138817` bytes in `1` second, which is about `1111 kbps` in encoded-video time.

## Requirements

- CMake 3.16 or newer
- A C++17 compiler
- OpenCV
- FFmpeg
- On Windows, Visual Studio 2022 or Visual Studio 2026 is recommended

## Build

### Visual Studio 2026

```powershell
Set-Location C:\path\to\CamCom-Binary
cmake -S . -B build-vs26 -G "Visual Studio 18 2026" -A x64 -DOpenCV_DIR="C:\path\to\opencv\build\x64\vc16\lib"
cmake --build build-vs26 --config Release
```

### Visual Studio 2022

```powershell
Set-Location C:\path\to\CamCom-Binary
cmake -S . -B build-vs22 -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="C:\path\to\opencv\build\x64\vc16\lib"
cmake --build build-vs22 --config Release
```

Build outputs:

```text
build-vs26\bin\Release\encode.exe
build-vs26\bin\Release\decode.exe
```

## Usage

### encode

```text
encode <input.bin> <output.mp4> <duration_milliseconds>
```

Example:

```powershell
.\build-vs26\bin\Release\encode.exe .\input.bin .\output.mp4 1000
```

Notes:

- The encoder now generates a strict `1000 ms` video for Project 1, even if a different positive duration is passed.
- The third argument is still accepted for CLI compatibility and is reported back as the requested duration.
- If the source file is larger than what fits in `1000 ms`, only the leading bytes are transmitted in the video.
- The stream header records the original source length, not only the transmitted prefix length.
- Bytes beyond the transmitted prefix are treated as zero-filled tail bytes during decode.

### decode

Three-argument mode:

```text
decode <input.mp4> <output.bin> <validity.bin>
```

Four-argument mode:

```text
decode <input.mp4> <output.bin> <validity.bin> <reference.bin>
```

Examples:

```powershell
.\build-vs26\bin\Release\decode.exe .\input.mp4 .\output.bin .\validity.bin
.\build-vs26\bin\Release\decode.exe .\input.mp4 .\output.bin .\validity.bin .\input.bin
```

Notes:

- In three-argument mode, `validity.bin` is a byte-validity mask for the recovered output.
- In four-argument mode, `validity.bin` is a byte-vote mask against the reference file.
- Bytes not actually recovered from the video are zero-filled in the output and marked as `0` in `validity.bin` to represent abstention.
- `compared bytes` counts only bytes that were actually recovered from video frames.
- `full accuracy` reports matched bytes over the full original message length, including abstained tail bytes.

## Encoding Layout

The frame layout uses:

- four large corner finder markers
- four small mid-edge markers
- a small quiet-zone border around the whole canvas
- a safety margin around the finder markers
- top reference color blocks placed inside the quiet zone
- a mask-based data area spread across the full canvas except for reserved marker/reference regions

The current renderer/decoder no longer limits payload to the inner rectangle only. Cells in the same rows and columns as the markers can be used for data if they do not overlap the reserved finder/reference shapes.

## Decoder Notes

The decoder currently includes several recovery stages:

- direct parse from each sampled frame
- grouped-sample recovery
- missing-frame recovery by scanning the video again
- payload voting for the same `frame_index` to avoid a bad early candidate locking in the wrong payload

This makes recorded-video decoding much more stable than the original direct-only flow.

## Important Defaults

Current relevant defaults in code:

- `fps = 20`
- `cell_size = 10`
- `cells_per_row = 219`
- `payload_bytes_per_frame = 7423`
- `reference_block_size = 2`
- FEC: `RS(255,239)`, interleave depth `4`

Because the first frame also carries bootstrap and stream-header bytes, its data payload is slightly smaller than later frames.

## Project Structure

```text
CamCom-Binary/
├─ include/
├─ src/
├─ CMakeLists.txt
└─ README.md
```

Main source files:

- `src/encoder.cpp`: encoder entry point
- `src/decoder.cpp`: decoder entry point
- `src/codec.cpp`: frame rendering, rectification, and sampling
- `src/fec.cpp`: FEC implementation
- `src/tracker.cpp`: tracking logic used by grouped decode

## Practical Advice

- First verify the generated video by decoding it directly before testing phone recordings.
- For phone recording tests, keep the screen square to the camera, reduce motion blur, and avoid aggressive HEVC/HDR processing if possible.
- If you compare accuracy for a short encoded clip, use the clip's transmitted byte window as the primary metric, not the full source file size.
