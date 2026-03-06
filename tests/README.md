# tests/

This directory contains sample files used for testing the encoder and decoder.

## Files

| File | Description |
|------|-------------|
| `sample_input.bin` | 32-byte test pattern (`0x00`–`0x1F`). Use as `<input.bin>` when running the encoder. |

## Suggested workflow

```bash
# From the build directory (after cmake --build):

# 1. Encode the sample file into a video (500 ms duration, 30 fps → 15 frames)
./bin/encoder ../tests/sample_input.bin /tmp/test_encoded.mp4 500

# 2. Play/display the video on screen, record it with a camera as recorded.mp4
#    (or pass the encoder output directly for a software loop-back test)

# 3. Decode the recorded video
./bin/decoder /tmp/test_encoded.mp4 /tmp/test_decoded.bin /tmp/test_validity.bin

# 4. Compare original and decoded bytes
cmp ../tests/sample_input.bin /tmp/test_decoded.bin && echo "PASS" || echo "MISMATCH"
```

> **Note:** Large recorded video files (`*.mp4`, `*.avi`, `recorded_*.bin`) are
> excluded from version control by `.gitignore`. Only small input test files are
> checked in.
