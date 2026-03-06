# Design Notes

> **Status:** Placeholder — to be filled in as the design is finalized.

## Goals

1. Encode an arbitrary binary file into a short video that can be played on a
   screen.
2. Capture that video with a smartphone or camera.
3. Decode the captured video back to the original bytes with high reliability.

## High-Level Architecture

```
Binary file
    │
    ▼
[Encoder]
    │  splits data into frames
    │  renders each frame as a grid of visual cells
    │  writes frames to video file (via FFmpeg)
    ▼
Video file (.mp4)
    │
    │  (physical transmission: play on screen, record with camera)
    ▼
Recorded video (.mp4)
    │
    ▼
[Decoder]
    │  decodes video frames (via FFmpeg)
    │  samples visual cells in each frame
    │  reassembles bytes from cells
    │  validates checksums; writes validity mask
    ▼
Recovered binary file + validity mask
```

## Key Design Decisions

### Visual Cell Encoding

Each data cell is a square block of pixels on screen.  The minimum cell size
is chosen so that even a low-resolution camera can reliably distinguish cells.

### Error Detection

Each frame carries a CRC-32 checksum of its payload.  The decoder marks any
frame with a checksum mismatch as invalid in the validity mask.

### Synchronisation

Every frame includes a per-frame header with a magic constant, version, and
frame index.  This allows the decoder to detect frame boundaries and recover
from dropped or reordered frames.

## TODO

- Evaluate error-correction codes (e.g., Reed-Solomon) for packet loss.
- Decide on colour vs. grayscale encoding trade-offs.
- Benchmark maximum reliable data rate vs. camera resolution / distance.
- Define robustness requirements (motion blur, perspective distortion).
