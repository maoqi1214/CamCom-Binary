// 实现 FFmpeg 可执行文件查找与外部进程调用辅助逻辑。
#include "ffmpeg.hpp"

#include <cstdlib>
#include <filesystem>
#include <sstream>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace camcom {
namespace fs = std::filesystem;

namespace {

std::string quote_arg(const std::string& value) {
    if (value.find_first_of(" \t\"") == std::string::npos) {
        return value;
    }

    std::string quoted = "\"";
    for (char ch : value) {
        if (ch == '"') {
            quoted += "\\\"";
        } else {
            quoted += ch;
        }
    }
    quoted += '"';
    return quoted;
}

std::string executable_dir(const char* argv0) {
    if (argv0 == nullptr || argv0[0] == '\0') {
        return {};
    }

    std::error_code ec;
    fs::path path = fs::absolute(fs::path(argv0), ec);
    if (ec) {
        return {};
    }
    return path.parent_path().string();
}

} // namespace

std::string find_ffmpeg_executable(const char* argv0) {
    const fs::path local_name =
#ifdef _WIN32
        "ffmpeg.exe";
#else
        "ffmpeg";
#endif

    const fs::path cwd_candidate = fs::current_path() / local_name;
    if (fs::exists(cwd_candidate)) {
        return cwd_candidate.string();
    }

    const std::string exe_dir = executable_dir(argv0);
    if (!exe_dir.empty()) {
        const fs::path exe_candidate = fs::path(exe_dir) / local_name;
        if (fs::exists(exe_candidate)) {
            return exe_candidate.string();
        }
    }

    return local_name.string();
}

int run_process(const std::vector<std::string>& args) {
    std::ostringstream command;
    bool first = true;
    for (const auto& arg : args) {
        if (!first) {
            command << ' ';
        }
        command << quote_arg(arg);
        first = false;
    }
    return std::system(command.str().c_str());
}

} // namespace camcom
