# CamCom-Binary

这是一个把二进制文件编码成视频、再从视频里还原二进制文件的 C++ 项目。

项目当前包含两个程序：

- `encoder.exe`：把 `input.bin` 编码成 `output.mp4`
- `decoder.exe`：把 `input.mp4` 解码回 `output.bin`

项目依赖：

- `OpenCV`：图像处理
- `ffmpeg.exe`：视频封装和抽帧
- `CMake + Visual Studio 2026`：本地构建

## 1. 需要先准备什么

在本地运行前，需要先装好下面 4 个东西：

### 1. Visual Studio 2026

安装时勾选：

- `Desktop development with C++`

### 2. CMake

安装完成后，在命令行里确认：

```bat
cmake --version
```

### 3. OpenCV

你最终需要拿到一个包含 `OpenCVConfig.cmake` 的目录。

常见路径示例：

```text
opencv\build\x64\vc16\lib
```

注意：

- 传给 `build_vs26.bat` 的不是 OpenCV 根目录
- 传的是 `OpenCVConfig.cmake` 所在目录
- 也就是上面这个 `...\lib` 目录

### 4. FFmpeg

这一项最容易配错，所以这里写清楚。

你们要下载的是：

- Windows 版 FFmpeg
- 下载后解压
- 最后要能看到 `ffmpeg.exe`

解压后通常会长这样：

```text
....ffmpeg\bin\ffmpeg.exe
或者 ....ffmpeg-master-latest-win64-gpl-shared\bin\ffmpeg.exe
```

这里真正要加到系统变量 `Path` 的，不是 `ffmpeg.exe` 文件本身，而是它所在的目录：

```text
....ffmpeg\bin
或者....ffmpeg-master-latest-win64-gpl-shared\bin
```

也就是说，你应该这样做：

1. 下载 FFmpeg 的 Windows 压缩包
2. 解压到一个固定目录，例如 `D:\tools\ffmpeg`
3. 确认 `ffmpeg.exe` 在 `D:\tools\ffmpeg\bin\ffmpeg.exe`
4. 把 `D:\tools\ffmpeg\bin` 加进系统环境变量 `Path`
5. 重新打开终端
6. 执行下面两条命令检查是否成功：

```bat
where ffmpeg
ffmpeg -version
```

如果能输出路径和版本号，就说明 FFmpeg 已经配好了。

## 2. 项目怎么编译

先克隆仓库：

```bat
git clone https://github.com/maoqi1214/CamCom-Binary.git
cd CamCom-Binary
```

然后在仓库根目录执行：

```bat
build_vs26.bat ......\opencv\build\x64\vc16\lib（前面省略号填写你的实际路径）
```

这里的参数要换成你自己机器上 `OpenCVConfig.cmake` 所在的目录（即bin目录）。

## 3. 编译完成后程序在哪

编译成功后，这两个程序在这里：

```text
build-vs26\bin\Debug\encoder.exe
build-vs26\bin\Debug\decoder.exe
```

## 4. 怎么运行

### 编码（在终端运行，以下input.bin,output.mp4,15是示例，实际使用时要替换,注意文件后缀）

```bat
build-vs26\bin\Debug\encoder.exe input.bin output.mp4 15
```

### 解码

```bat
build-vs26\bin\Debug\decoder.exe input.mp4 recovered.bin
```

### 可选：和原文件做对比（即decoder有两种编译方式，一个是传两个参数，一个是传三个参数）

```bat
build-vs26\bin\Debug\decoder.exe input.mp4 recovered.bin input.bin
```
（传三个参数时会多输出一个对比文件v1.bin，用于老师的测评代码）
## 5. 手机录像时的注意事项

如果你是先播放视频，再用手机录屏幕，建议注意：

- 屏幕内容要拍全
- 尽量避免反光
- 尽量避免自动曝光剧烈变化
- 视频最后多停 1 到 2 秒再结束录制
- 不要让播放器控制条挡住最后几帧

## 6. 最关键的两个路径

### OpenCV 路径

构建脚本参数传这个目录：

```text
D:\vscode\light\opencv\build\x64\vc16\lib
```

### FFmpeg 路径

系统变量 `Path` 里加这个目录：

```text
D:\tools\ffmpeg\bin
```


## 7. 配置完成后自检

配完环境后，可以先检查这三条命令：

```bat
cmake --version
where ffmpeg
ffmpeg -version
```

如果这三条都正常，再开始执行项目构建。

## 8. 如果你用的是 VS2022

如果你本机安装的是 Visual Studio 2022，那么 `build_vs26.bat` 里的生成器不能直接用。

默认脚本现在写的是：

```bat
Visual Studio 18 2026
```

VS2022 对应的生成器是：

```bat
Visual Studio 17 2022
```

你有两种做法：

### 做法 1：直接改 `build_vs26.bat`

把脚本里这一行：

```bat
cmake -S . -B build-vs26 -G "Visual Studio 18 2026" -A x64 -DOpenCV_DIR="%OPENCV_DIR%"
```

改成：

```bat
cmake -S . -B build-vs26 -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="%OPENCV_DIR%"
```

然后再执行：

```bat
build_vs26.bat 你的 OpenCVConfig.cmake 所在目录
```

### 做法 2：不改脚本，直接在终端执行

在仓库根目录执行：

```bat
cmake -S . -B build-vs22 -G "Visual Studio 17 2022" -A x64 -DOpenCV_DIR="你的 OpenCV lib 目录"
cmake --build build-vs22 --config Debug
```

编译成功后，可执行文件在：

```text
build-vs22\bin\Debug\encoder.exe
build-vs22\bin\Debug\decoder.exe
```

如果是 VS2022，优先推荐做法 2，不会影响别人已经在用的 `build_vs26.bat`。
