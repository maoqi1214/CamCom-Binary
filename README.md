# Screen-to-Camera Binary Transmission System

This project implements a system for transmitting binary data from a screen to a camera using video encoding techniques.

## Goals
- Develop a reliable binary transmission system using OpenCV and FFmpeg.
- Ensure data integrity and validity through proper handling of video frames.

## Dependencies
### Windows
1. Install OpenCV: [Installation Guide](https://opencv.org/releases/)
2. Install FFmpeg: [Installation Guide](https://ffmpeg.org/download.html)

### Linux
1. Install OpenCV: `sudo apt install libopencv-dev`
2. Install FFmpeg: `sudo apt install ffmpeg`

### macOS
1. Install OpenCV: `brew install opencv`
2. Install FFmpeg: `brew install ffmpeg`

## Build Instructions
1. Clone the repository
2. Create a build directory:
   ```bash
   mkdir build && cd build
   ```
3. Run CMake:
   ```bash
   cmake ..
   ```
4. Build the project:
   ```bash
   make
   ```

## Usage Examples
To encode a binary file to video:
```bash
./encoder <input.bin> <output.mp4> <duration_ms>
```

To decode a video back to binary:
```bash
./decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

## Validity Mask Format
The `vout.bin` format for the validity mask will track the validity of each bit in the output binary file, helping to identify erroneous bits for correction and adjustment.
