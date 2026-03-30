# CamCom-Binary

`CamCom-Binary` 是一个基于 C++、OpenCV 和 FFmpeg 的可见光传输实验项目。它将二进制文件编码为一系列彩色码帧并生成视频，再从屏幕录制或手机拍摄的视频中恢复出原始二进制数据。

项目当前提供两个命令行程序：

- `encode`：将二进制文件编码为视频
- `decode`：从视频中解码出二进制文件

## 特性

- 保留彩色编码，每个单元格承载 2 bit 信息
- 固定 `15 FPS` 输出
- 使用四角定位点进行检测和透视矫正
- 支持原始生成视频回解，也支持手机录制视频解码
- 解码端包含分组采样、QuadTracker 跟踪和多候选恢复逻辑
- 数据帧启用分块 `RS(255,223)` 帧内纠错与交织

## 依赖

- CMake 3.16 及以上
- 支持 C++17 的编译器
- OpenCV
- FFmpeg
- Windows 下推荐使用 Visual Studio 2026 或 Visual Studio 2022

## 构建

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

构建完成后可执行文件默认位于：

```text
build-vs26\bin\Release\encode.exe
build-vs26\bin\Release\decode.exe
```

或：

```text
build-vs22\bin\Release\encode.exe
build-vs22\bin\Release\decode.exe
```

## 用法

### encode

```text
encode <input.bin> <output.mp4> <max_milliseconds>
```

示例：

```powershell
.\build-vs26\bin\Release\encode.exe .\input.bin .\output.mp4 15000
```

说明：

- 第一个参数：输入二进制文件
- 第二个参数：输出视频路径
- 第三个参数：允许生成视频的最大时长，单位毫秒
- 当前编码器固定输出 `15 FPS`

### decode

三参数模式：

```text
decode <input.mp4> <output.bin> <validity.bin>
```

四参数模式：

```text
decode <input.mp4> <output.bin> <validity.bin> <reference.bin>
```

示例：

```powershell
.\build-vs26\bin\Release\decode.exe .\input.mp4 .\output.bin .\validity.bin
.\build-vs26\bin\Release\decode.exe .\input.mp4 .\output.bin .\validity.bin .\input.bin
```

说明：

- 三参数模式下，`validity.bin` 表示逐字节恢复有效性
- 四参数模式下，`validity.bin` 表示与参考文件逐字节比较后的正确性掩码

## 编码方式概述

编码器会生成以下几类帧：

- `bootstrap`：编码参数信息
- `stream header`：总字节数、总帧数等流信息
- 数据帧：实际二进制载荷
- 重复帧：用于提升尾部恢复稳定性

每个字节会拆成 4 个 2-bit 符号，并映射到彩色单元格中。当前实现保留彩色编码，不是黑白二维码方案。

## 默认参数

当前代码中的默认参数为：

- 输出帧率：`15 FPS`
- `cell_size`：`10`
- 每行单元数：`219`
- 每列单元数：`112`
- 码面 payload 容量：`6100 byte/frame`
- 用户有效载荷：`5332 byte/frame`
- 理论有效传输速率：`5332 x 15 = 79980 byte/s`，约为 `80.0 KB/s`，即 `640 kbps`
- 纠错参数：`RS(255,223)`，交织深度 `4`

其中每帧数据字节数的来源如下：

- 当前数据区共有 `219 x 112 = 24528` 个单元格
- 每个单元格表示 `2 bit`
- `4` 个单元格组成 `1 byte`
- 因此整帧最多约可表示 `24528 / 4 = 6132` 字节
- 数据帧头固定占用 `4 + 1 + 4 + 4 + 4 + 4 = 21` 字节
- 所以理论上的有效载荷上限约为 `6132 - 21 = 6111` 字节
- 当前代码将码面 payload 容量设置为 `6100` 字节，保留少量余量以提高布局和解码稳定性
- 由于启用了 `RS(255,223)`，`6100` 字节中并不全是用户数据
- 按当前纠错参数换算后，每帧实际可承载的用户有效载荷约为 `5332` 字节

## 项目结构

```text
CamCom-Binary/
├─ include/
├─ src/
├─ CMakeLists.txt
└─ README.md
```

主要源码文件：

- `src/encoder.cpp`：编码器入口
- `src/decoder.cpp`：解码器入口
- `src/codec.cpp`：渲染、定位、矫正、采样
- `src/tracker.cpp`：跟踪与 Kalman 相关逻辑

## 使用建议

- 先对原始生成视频做一次直接回解，确认编解码链路本身正常
- 再进行全屏播放和手机录制测试
- 拍摄时尽量保证码面完整入镜，避免强反光和曝光剧烈变化
