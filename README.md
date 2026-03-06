# Project Goals

- Implement a framework for audio/video encoding and decoding.
- Utilize OpenCV and FFmpeg for media handling.
- Ensure reliability and performance through testing.

# Dependency Installation

## Windows
- Install OpenCV: [OpenCV Installation Guide](https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html)
- Install FFmpeg: [FFmpeg Installation Guide](https://ffmpeg.org/download.html)

## Linux
- Install OpenCV: `sudo apt-get install libopencv-dev`
- Install FFmpeg: `sudo apt-get install ffmpeg`

## macOS
- Install OpenCV: `brew install opencv`
- Install FFmpeg: `brew install ffmpeg`

# Build Using CMake

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```
2. Run CMake:
   ```bash
   cmake ..
   ```
3. Compile the project:
   ```bash
   make
   ```

# Usage Examples

To encode: `./encoder <input.bin> <output.mp4> <duration_ms>`
To decode: `./decoder <recorded.mp4> <output.bin> <validity_mask.bin>`

# Validity Mask Format

The file `vout.bin` contains a validity mask indicating the validity of each frame. Each byte represents a frame's validity, where `0x00` = invalid and `0x01` = valid.