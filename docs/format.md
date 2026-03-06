# Binary File Format

> **Status:** Placeholder — to be filled in as the implementation matures.

## Overview

The CamCom-Binary format encodes arbitrary binary data into a sequence of video
frames that can be captured by a camera and later decoded back to the original
bytes.

## Stream Structure

```
┌─────────────────────────────────┐
│        Stream Header            │  written once, embedded in frame 0
├─────────────────────────────────┤
│  Frame 0: FrameHeader + payload │
├─────────────────────────────────┤
│  Frame 1: FrameHeader + payload │
│            …                    │
└─────────────────────────────────┘
```

## Stream Header Fields

| Field              | Type     | Description                               |
|--------------------|----------|-------------------------------------------|
| `magic`            | uint32   | Always `0x43414D43` ("CAMC")              |
| `version`          | uint8    | Format version (currently `1`)            |
| `total_data_bytes` | uint64   | Size of the original (unencoded) file     |
| `encoding`         | uint8    | Encoding scheme (0 = Binary, 1 = Gray4)   |
| `fps`              | uint32   | Target playback frame-rate                |
| `cell_size`        | uint32   | Size in pixels of each visual data cell   |

## Per-Frame Header Fields

| Field           | Type   | Description                                     |
|-----------------|--------|-------------------------------------------------|
| `magic`         | uint32 | Always `0x43414D43`                             |
| `version`       | uint8  | Must match stream header version                |
| `frame_index`   | uint32 | 0-based index of this frame                     |
| `total_frames`  | uint32 | Total number of data frames in this stream      |
| `payload_bytes` | uint32 | Number of payload bytes in this frame           |
| `checksum`      | uint32 | CRC-32 of this frame's payload bytes            |

## Validity Mask

The decoder outputs a `validity_mask.bin` file with one byte per payload byte.
A value of `0x01` means the byte was recovered confidently; `0x00` means the
decoder could not determine the value (e.g., due to motion blur or occlusion).

## TODO

- Define exact on-screen layout (grid dimensions, border, sync markers).
- Define error-correction layer (e.g., Reed-Solomon).
- Specify how multi-channel (color) encoding is handled in Gray4 mode.
