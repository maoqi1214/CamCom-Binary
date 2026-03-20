// FFmpeg executable lookup and external process invocation interfaces.
#pragma once

#include <string>
#include <vector>

namespace camcom {

std::string find_ffmpeg_executable(const char* argv0);
int run_process(const std::vector<std::string>& args);

} // namespace camcom
