# C++17 Project

This project implements an encoder and decoder based on the Nyquist-Shannon sampling theorem. It includes functionality for encoding audio data into video format and decoding it back.

## Nyquist/Shannon Sampling Theorem

The Nyquist-Shannon theorem establishes the minimum sampling rate required to accurately reconstruct a signal from its samples.

## Installation

### Windows
- Install necessary dependencies through [Vcpkg](https://github.com/microsoft/vcpkg).

### Linux
- Use package manager to install dependencies:
```
sudo apt install <dependencies>
```

### macOS
- Install dependencies using Homebrew:
```
brew install <dependencies>
```

## Build

To build the project, use CMake:
```
mkdir build
cd build
cmake ..
make
```

## Usage Examples
- To encode:
```
./encoder <input.bin> <output.mp4> <duration_ms>
```

- To decode:
```
./decoder <recorded.mp4> <output.bin> <validity_mask.bin>
```

## Validity Mask Format
The validity mask is a binary file indicating the validity of each bit in the output.
