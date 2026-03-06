# Tests

This directory contains test assets and describes how to run smoke tests for
CamCom-Binary.

## Contents

| File | Description |
|------|-------------|
| `sample_input.bin` | A tiny binary file (16 bytes) used for smoke testing the encoder and decoder. |
| `README.md` | This file. |

## Running smoke tests

After building the project (see the root `README.md`), the encoder and decoder
binaries are placed in `build/bin/`.

### Encoder smoke test

```bash
build/bin/encoder tests/sample_input.bin /tmp/test_out.mp4 1000
```

Expected output (skeleton / TODO stage):

```
[encoder] Input : tests/sample_input.bin
[encoder] Output: /tmp/test_out.mp4
[encoder] Duration (ms): 1000
[encoder] TODO: encoding not yet implemented.
```

### Decoder smoke test

```bash
build/bin/decoder /tmp/test_out.mp4 /tmp/recovered.bin /tmp/mask.bin
```

Expected output (skeleton / TODO stage):

```
[decoder] Video : /tmp/test_out.mp4
[decoder] Output: /tmp/recovered.bin
[decoder] Mask  : /tmp/mask.bin
[decoder] TODO: decoding not yet implemented.
```

> **Note:** Until the TODO implementations are complete the encoder will not
> produce a real video file, so the decoder will report that the file is not
> found when given `/tmp/test_out.mp4`.

## Adding new tests

- Place small binary fixtures (≤ a few KB) directly in this directory.
- Larger test videos should **not** be committed; generate them locally or
  download them as part of a CI artefact step.
