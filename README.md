# CamCom-Binary

基于 `C++ / OpenCV / FFmpeg` 的可见光二进制传输实验项目。

本版本重点满足项目一的提交要求：

- 输入文件总长度会写入流头
- 按给定时长生成视频，当前主要使用 `1000 ms`
- 输出视频固定为 `20 FPS`
- 在 1 秒内尽可能传输更多前缀数据
- 如果 1 秒内传不完，剩余部分按 `0` 补齐
- `vote / validity` 中对应未传输部分写 `0x00`，表示弃权

项目一容忍错误率：

```c
#define TORRENT_ERR_RATE 0.0003
```

## 环境

- CMake 3.16+
- 支持 C++17 的编译器
- OpenCV
- FFmpeg

Windows 下建议使用 Visual Studio。

## 编译

如果已经有 `build-vs26`：

```powershell
cmake --build build-vs26 --config Release
```

如果需要首次生成工程：

```powershell
cmake -S . -B build-vs26 -G "Visual Studio 18 2026" -A x64
cmake --build build-vs26 --config Release
```

生成的可执行文件位于：

```text
build-vs26\bin\Release\encode.exe
build-vs26\bin\Release\decode.exe
```

建议后续所有命令都在 `build-vs26\bin\Release` 目录下执行：

```powershell
cd .\build-vs26\bin\Release
```

## 基本操作

### 1. 生成 1 秒视频

命令格式：

```text
encode <输入文件> <输出视频> <时长毫秒>
```

示例：

```powershell
encode.exe 01.bin in1.mp4 1000
```

含义：

- 把 `01.bin` 编码成 `in1.mp4`
- 视频时长严格为 `1000 ms`
- 当前固定输出为 `20 FPS`

### 2. 直接回解生成的视频

命令格式：

```text
decode <输入视频> <输出文件> <vote文件> <参考文件>
```

示例：

```powershell
decode.exe in1.mp4 out.bin out.vote.bin 01.bin
```

输出含义：

- `out.bin`：解码得到的结果
- `out.vote.bin`：与输出等长的投票文件
- `01.bin`：参考文件，用来计算正确率

### 3. 解码录屏或实拍视频

如果你录下了播放过程，例如得到 `out1.mp4`，可以这样解码：

```powershell
decode.exe out1.mp4 out1.bin out1.vote.bin 01.bin
```

其中：

- `out1.mp4`：录屏或拍摄后得到的视频
- `out1.bin`：恢复出的数据
- `out1.vote.bin`：每个字节是否有效
- `01.bin`：原始参考文件

### 4. 批量生成 1 秒视频

如果目录下有 `01.bin` 到 `10.bin`，可以逐个生成：

```powershell
encode.exe 01.bin in1.mp4 1000
encode.exe 02.bin in2.mp4 1000
encode.exe 03.bin in3.mp4 1000
encode.exe 04.bin in4.mp4 1000
encode.exe 05.bin in5.mp4 1000
encode.exe 06.bin in6.mp4 1000
encode.exe 07.bin in7.mp4 1000
encode.exe 08.bin in8.mp4 1000
encode.exe 09.bin in9.mp4 1000
encode.exe 10.bin in10.mp4 1000
```

### 5. 批量解码录屏结果

如果你录完后得到 `out1.mp4` 到 `out10.mp4`，可以按对应关系分别解码：

```powershell
decode.exe out1.mp4 out1.bin out1.vote.bin 01.bin
decode.exe out2.mp4 out2.bin out2.vote.bin 02.bin
decode.exe out3.mp4 out3.bin out3.vote.bin 03.bin
decode.exe out4.mp4 out4.bin out4.vote.bin 04.bin
decode.exe out5.mp4 out5.bin out5.vote.bin 05.bin
decode.exe out6.mp4 out6.bin out6.vote.bin 06.bin
decode.exe out7.mp4 out7.bin out7.vote.bin 07.bin
decode.exe out8.mp4 out8.bin out8.vote.bin 08.bin
decode.exe out9.mp4 out9.bin out9.vote.bin 09.bin
decode.exe out10.mp4 out10.bin out10.vote.bin 10.bin
```

### 6. 如何看解码结果

程序会输出几项核心信息：

- `decoded frames`：成功恢复了多少个数据帧
- `total frames`：理论总数据帧数
- `compared bytes`：实际参与比较的字节数
- `matched bytes`：与参考文件一致的字节数
- `accuracy`：只统计参与投票部分的准确率
- `full accuracy`：按整个原始文件统计的准确率

如果 `accuracy = 100%`，说明已经成功恢复出的那部分数据是完全正确的。
如果 `full accuracy` 很低，通常表示 1 秒内只传输了原文件的一部分，这在当前规则下是正常的。

## 编码

命令：

```text
encode <input.bin> <output.mp4> <duration_ms>
```

示例：

```powershell
.\build-vs26\bin\Release\encode.exe .\input.bin .\out.mp4 1000
```

说明：

- `1000` 表示严格生成 `1 秒` 视频
- 当前编码参数固定为 `20 FPS`
- 视频只传前面能装下的数据
- 流头记录原始输入文件总长度

## 解码

命令：

```text
decode <input.mp4> <output.bin> <validity.bin>
decode <input.mp4> <output.bin> <validity.bin> <reference.bin>
```

示例：

```powershell
.\build-vs26\bin\Release\decode.exe .\out.mp4 .\output.bin .\vote.bin
.\build-vs26\bin\Release\decode.exe .\out.mp4 .\output.bin .\vote.bin .\input.bin
```

说明：

- `output.bin` 为恢复结果
- `validity.bin` / `vote.bin` 与输出等长
- 成功恢复的位置写 `0xFF`
- 未恢复或未传输的位置写 `0x00`

## 正确率含义

提供参考文件时，程序会输出两类指标：

- `accuracy`：只统计参与投票的字节
- `full accuracy`：按整个原始文件长度统计

因此，当 1 秒内只能传输一部分数据时：

- `accuracy` 可能很高
- `full accuracy` 会明显更低

这是当前设计下的正常现象。

## 当前默认参数

- `fps = 20`
- `cell_size = 10`
- `cells_per_row = 219`
- `reference_block_size = 2`
- `payload_bytes_per_frame = 6100`

## 主要源码

- `src/encoder.cpp`：编码入口
- `src/decoder.cpp`：解码入口
- `src/codec.cpp`：画面渲染与采样
- `src/fec.cpp`：FEC 编解码
- `src/ffmpeg.cpp`：FFmpeg 调用

## 目录

```text
CamCom-Binary/
|-- include/
|-- src/
|-- CMakeLists.txt
`-- README.md
```
