# CamCom-Binary 团队环境配置与运行指南（Windows）

> 适用于：从 GitHub 下载源码后，在本地完成 `OpenCV + FFmpeg + CMake` 配置、编译并运行 `encoder/decoder`。

---

## 1. 源码获取

项目仓库：`https://github.com/maoqi1214/CamCom-Binary`


### 方式 A：Git 克隆（推荐）
```powershell
git clone https://github.com/maoqi1214/CamCom-Binary.git
cd CamCom-Binary
```

---

## 2. 依赖与版本说明

### 2.1 项目最低要求
- CMake：`>= 3.16`
- C++ 标准：`C++17`
- 编译依赖：`OpenCV`
- 运行依赖：`FFmpeg`（程序通过命令行调用 `ffmpeg.exe`）

### 2.2 已验证环境（可作为团队参考基线）
- Visual Studio：`Microsoft Visual Studio Community 2026 (18.4.1)`
- CMake：`4.2.3-msvc3`
- OpenCV：`4.13.0`（MSVC 体系，路径示例：`...\x64\vc16\lib`）
- FFmpeg：`N-123250-gb8a4d8a18d-20260307`

> 说明：FFmpeg 版本可不同，通常不影响本项目使用，只要 `ffmpeg -version` 正常。(可以在系统终端（win+R后输入CMD）输入ffmpeg -version以验证)

---

## 3. 工具链兼容性（最重要）

请保证 **编译器体系与 OpenCV 二进制匹配**：

- `OpenCV vc16 / vc17` 这类是 **MSVC** 版本，只能配 `cl.exe`（Visual Studio）。


如果混用（例如 `g++` 链接 `vc16`），会出现链接错误（如 `undefined reference`）。

---

## 4. FFmpeg 配置（必须）

假设你的 `ffmpeg.exe` 在：
`D:\tools\ffmpeg\bin\ffmpeg.exe`

把 `D:\tools\ffmpeg\bin` 加入 `PATH`。

### 图形界面配置
1. `此电脑` -> `属性` -> `高级系统设置` -> `环境变量`
2. 在“用户变量”或“系统变量”里编辑 `Path`
3. 新增 `D:\tools\ffmpeg\bin`
4. 重新打开终端 / Visual Studio

### 验证
```powershell
where.exe ffmpeg
ffmpeg -version
```

---

## 5. OpenCV 配置（MSVC 方案）

确保目录里有 `OpenCVConfig.cmake`，例如：
`D:\vscode\light\opencv\build\x64\vc16\lib\OpenCVConfig.cmake`

CMake 配置时传：
- `-DOpenCV_DIR="D:\vscode\light\opencv\build\x64\vc16\lib"`
- （改成自己的lib路径）

---

## 6. 一次性完整构建步骤

在 `PowerShell` 执行：

```powershell
chcp 65001
cd D:\vscode\light\CamCom-Binary

if (Test-Path build) { Remove-Item build -Recurse -Force }

cmake -S . -B build -G "Visual Studio 18 2026" -A x64 -DOpenCV_DIR="D:\vscode\light\opencv\build\x64\vc16\lib"
cmake --build build --config Release -j
```

> 如果你是 VS2022，也可以用：
> `-G "Visual Studio 17 2022"`

---

## 7. 运行方式

编译成功后可执行文件通常在：
`build\bin\Release\`

### 7.1 运行 encoder
```powershell
cd D:\vscode\light\CamCom-Binary\build\bin\Release
.\encoder.exe <input.bin> <output.mp4> <fps>
```

示例：
```powershell
.\encoder.exe ..\..\..\tests\sample_input.bin out.mp4 10
```

> 注意：第二个参数必须是视频文件名（例如 `.mp4`），不能是 `.bin`。

### 7.2 运行 decoder
```powershell
.\decoder.exe <input.mp4> <output.bin> [reference_input.bin]
```

示例：
```powershell
.\decoder.exe out.mp4 recovered.bin ..\..\..\tests\sample_input.bin
```

---

## 8. 常见问题排查

### 问题 1：
`Unable to choose an output format for 'out.bin'`

原因：`encoder` 输出参数写成了 `.bin`。

修复：改成 `.mp4`，如 `out.mp4`。

---

### 问题 2：
`LNK4272: x64 与 x86 冲突`

原因：工程按 x86 构建，但 OpenCV 是 x64。

修复：使用 `-A x64`，并删除旧 `build` 后重新配置。

---

### 问题 3：
`undefined reference to cv::...`

原因：工具链混用（如 `g++` + `OpenCV vc16`）。

修复：
- 要么全用 MSVC；
- 要么改用 MinGW 版 OpenCV + g++。

---

### 问题 4：
`ffmpeg command failed`

原因：`ffmpeg` 不在 PATH 或参数错误。

修复：
```powershell
where.exe ffmpeg
ffmpeg -version
```
确认可执行后再跑。

---

## 9. 建议团队统一配置（减少踩坑）

建议统一使用：
- Visual Studio（MSVC）
- `x64` 架构
- OpenCV（MSVC 对应版本）
- FFmpeg（任意可用版本，PATH 已配置）

这样最稳定，也最容易复现。
