// 声明 FFmpeg 可执行文件查找与外部进程调用辅助接口。
#pragma once

#include <string>
#include <vector>

namespace camcom {

std::string find_ffmpeg_executable(const char* argv0);
int run_process(const std::vector<std::string>& args);

} // namespace camcom
