# Windows Build Guide

This guide is for teammates who are not familiar with CMake.

## 0. Clone The Repository

```bat
git clone https://github.com/maoqi1214/CamCom-Binary.git
cd CamCom-Binary
```

## 1. Install Tools

Install these first:

- Visual Studio 2026 with Desktop development with C++
- CMake
- OpenCV for Windows x64
- FFmpeg

## 2. Prepare OpenCV

You need the path that contains `OpenCVConfig.cmake`.

Typical example:

```text
D:\vscode\light\opencv\build\x64\vc16\lib
```

## 3. Build With One Command

Open `cmd` or PowerShell in the repository root and run:

```bat
build_vs26.bat D:\vscode\light\opencv\build\x64\vc16\lib
```

This script does two things:

1. generates the Visual Studio project with CMake
2. builds the Debug configuration

## 4. Where The Programs Are

After the build finishes:

```text
build-vs26\bin\Debug\encoder.exe
build-vs26\bin\Debug\decoder.exe
```

Put `ffmpeg.exe` in the same folder if it is not already on `PATH`.

## 5. Run Encoder

```bat
build-vs26\bin\Debug\encoder.exe input.bin output.mp4 15
```

## 6. Run Decoder

```bat
build-vs26\bin\Debug\decoder.exe input.mp4 recovered.bin
```

If you also want an accuracy report:

```bat
build-vs26\bin\Debug\decoder.exe input.mp4 recovered.bin input.bin
```

## 7. Common Problems

### CMake cannot find OpenCV

The OpenCV path is wrong. Pass the directory that contains `OpenCVConfig.cmake`.

### `ffmpeg` failed

Put `ffmpeg.exe` next to `encoder.exe` and `decoder.exe`, or add it to `PATH`.

### Visual Studio generator not found

This project is currently set up for:

```text
Visual Studio 18 2026
```

If your Visual Studio version is different, update `build_vs26.bat`.
