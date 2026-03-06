# Screen-to-Camera Binary Transmission System

## Goals
The goal of this project is to implement a binary transmission system that transmits binary data from a screen to a camera. This project will explore the following areas:
- **Physical Layer**: Develop understanding of how binary data is transmitted over physical mediums.
- **Nyquist/Shannon Theorem**: Analyze the limits of data transmission and optimize our system accordingly.

## Dependency Installation

### Windows
1. Install [CMake](https://cmake.org/download/).
2. Install [OpenCV](https://opencv.org/releases/) and [FFmpeg](https://ffmpeg.org/download.html).

### Linux
```bash
sudo apt-get install cmake libopencv-dev libavcodec-dev libavformat-dev libavutil-dev
```

### macOS
```bash
brew install cmake opencv ffmpeg
```

## CMake Build Steps
1. Navigate to the project root.
2. Run `mkdir build && cd build`.
3. Run `cmake ..`.
4. Run `make`.

## Usage Examples
To encode binary data:
```bash
./encoder <input.bin> <output.mp4> <duration_ms>
```

To decode binary data:
```bash
./decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

## Validity Mask Definition
The validity mask is defined as one byte per decoded bit:
- `0x01`: Valid bit
- `0x00`: Invalid bit
- MSB-first bit ordering.
