# CamCom-Binary

`CamCom-Binary` 是一个基于 C++ 的“屏幕到相机”二进制传输系统：
它将任意二进制文件编码为视频帧（含定位标记、帧头、CRC、Reed-Solomon 冗余），并在解码端从视频中恢复原始字节流。

当前代码已具备可运行的编码与解码流程，不再是仅含 TODO 的骨架工程。

## 1. 项目概述

核心功能：

- 将输入二进制文件切分为多帧数据并编码为视频。
- 使用四角定位标记 + 透视矫正 + 网格采样恢复数据。
- 使用 `CRC-32` 做帧级校验，使用 `Reed-Solomon` 做纠错。
- 通过 `bootstrap` 帧和 `stream header` 帧传输解码参数，降低配置耦合。
- 支持端到端测试与指标统计（BER、吞吐、帧成功率）。

主要特性：

- 高鲁棒性数据帧设计：数据帧之间插入黑帧以降低运动模糊影响。
- 码流自描述：解码端可先读取 `bootstrap` 自动获知参数。
- 视觉编码采用 2-bit 单元（4 色映射），提升单帧信息密度。
- 工程结构清晰：核心库 `camcom_core` + 多个可执行目标（`encoder`、`decoder`、测试程序）。

## 2. 技术栈

| 分类 | 技术 |
|---|---|
| 前端 | 无（命令行应用，不含 Web/UI 前端） |
| 后端 | C++、OpenCV（图像处理/视频读写）、FFmpeg 命令行（图像序列与视频互转） |
| 数据库 | 无 |
| 开发工具 | CMake、Ninja、Visual Studio/CMake 工作流 |

说明：项目不直接链接 FFmpeg 开发库，而是通过系统 `ffmpeg` 可执行文件完成封装/拆帧。

## 3. 项目架构

整体为“编码端 + 传输介质 + 解码端”的离线链路架构。

数据流向：

1. `encoder` 读取原始二进制文件。
2. 生成 `bootstrap` 与 `stream header`，再按帧打包业务数据（帧头 + payload + RS）。
3. `codec` 将每帧数据渲染为带定位标记的图像序列。
4. 通过 `ffmpeg` 将图像序列编码成视频。
5. `decoder` 先用 `ffmpeg` 抽帧，再对每帧做采样与解析。
6. 依次识别 `bootstrap`、`stream header`、数据帧，做 RS 解码与 CRC 校验。
7. 重组输出二进制文件；可选与参考文件对比并输出精度报告。

模块关系：

- `src/encoder.cpp`、`src/decoder.cpp`：两个入口程序。
- `src/codec.cpp`：视觉渲染与网格采样核心。
- `src/rs.cpp`：GF(256) + RS 编解码实现。
- `src/common.cpp`：CRC-32 与公共常量/结构。
- `src/io.cpp`：二进制文件读写工具。

## 4. 目录结构

以下目录树基于当前仓库实际文件整理（已忽略 `.git`、`node_modules`、`venv`、`dist` 等无关目录）：

```text
CamCom-Binary/
├── CMakeLists.txt
├── CMakeSettings.json
├── README.md
├── LICENSE
├── docs/
│   ├── design.md
│   ├── format.md
│   └── usage.md
├── include/
│   ├── codec.hpp
│   ├── common.hpp
│   ├── io.hpp
│   ├── rs.hpp
│   └── tracker.hpp
├── src/
│   ├── codec.cpp
│   ├── common.cpp
│   ├── decoder.cpp
│   ├── encoder.cpp
│   ├── io.cpp
│   ├── rs.cpp
│   └── tracker.cpp
├── tests/
│   ├── README.md
│   ├── integration_metrics.cpp
│   ├── integration_test.cpp
│   ├── rs_unit_test.cpp
│   ├── payload.bin
│   └── sample_input.bin
├── out.bin
├── test.bin
└── temp_frames_dec/
    └── frame_*.png
```

目录说明：

- `include/`：核心接口与数据结构定义，是跨模块协作边界。
- `src/`：核心业务逻辑实现，包含编码、解码、纠错、采样等关键流程。
- `tests/`：单元测试与集成测试程序及测试样本。
- `docs/`：设计和格式文档（其中部分内容仍保留早期占位描述，建议后续同步更新）。

## 5. 核心文件说明

### 5.1 项目入口文件和配置文件

- `CMakeLists.txt`
  - 定义 `camcom_core` 核心库。
  - 构建可执行文件：`encoder`、`decoder`、`rs_unit_test`、`integration_test`、`integration_metrics`。
  - 依赖 `OpenCV`，并设置 `encoder/decoder` 输出到 `build/bin`。
- `src/encoder.cpp`
  - 编码端主入口：读文件、分帧、生成帧头与校验、渲染图像并调用 `ffmpeg` 生成视频。
- `src/decoder.cpp`
  - 解码端主入口：抽帧、采样、解析帧类型、纠错与校验、重组输出。

### 5.2 核心业务逻辑实现

- `src/codec.cpp`
  - `render_frame`：将字节序列映射到彩色网格，叠加四角定位标记与辅助标记。
  - `sample_frame`：支持 raw/warp/crop 多路径采样回退，提高复杂画面下恢复率。
  - `warp_with_finders`：基于轮廓候选检测定位块并执行透视变换。
- `src/rs.cpp`
  - 完整实现 GF(256) 运算、RS 编码与解码流程（含 Berlekamp-Massey、Forney 等）。
- `src/common.cpp`
  - CRC-32 计算实现，用于帧 payload 完整性校验。

### 5.3 数据模型和 API 接口

- `include/common.hpp`
  - 定义常量（`MAGIC`、`FORMAT_VERSION` 等）、退出码枚举、编码方式枚举。
  - 定义 `FrameHeader`、`StreamHeader` 两类关键数据模型。
- `include/codec.hpp`
  - 定义 `EncoderConfig` 与视觉编解码接口（渲染、采样、清晰度评估、颜色标定）。
- `include/rs.hpp`
  - 定义 RS 编解码公开接口。
- `include/io.hpp`
  - 定义二进制读写和文件存在性/大小查询接口。

### 5.4 关键组件和服务模块

- `src/io.cpp`：文件读写基础设施。
- `src/tracker.cpp` + `include/tracker.hpp`：四角点 Kalman 跟踪组件（可作为解码端稳态跟踪能力基础）。
- `tests/integration_test.cpp`：从模拟编码到模拟采集再解码的端到端验证。
- `tests/integration_metrics.cpp`：输出 BER、吞吐率、帧成功率等指标，支持性能与可靠性评估。

## 构建与运行

构建：

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

运行：

```bash
# 编码
build/bin/encoder <input.bin> <output.mp4> <fps>

# 解码
build/bin/decoder <input.mp4> <output.bin> [reference_input.bin]
```

示例：

```bash
build/bin/encoder tests/sample_input.bin out.mp4 10
build/bin/decoder out.mp4 recovered.bin tests/sample_input.bin
```
