#include "io.hpp"

#include <fstream>
#include <stdexcept>
#include <sys/stat.h>

namespace camcom {

// ---------------------------------------------------------------------------
// Binary file I/O
// ---------------------------------------------------------------------------

std::vector<uint8_t> read_binary_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for reading: " + path);
    }
    const std::streampos end_pos = file.tellg();
    if (end_pos < 0) {
        throw std::runtime_error("Failed to determine size of file: " + path);
    }
    const auto size = static_cast<std::size_t>(end_pos);
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buffer(size);
    if (size > 0 && !file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(size))) {
        throw std::runtime_error("Failed to read file: " + path);
    }
    return buffer;
}

void write_binary_file(const std::string& path, const std::vector<uint8_t>& data) {
    write_binary_file(path, data.data(), data.size());
}

void write_binary_file(const std::string& path, const uint8_t* data, std::size_t size) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file for writing: " + path);
    }
    if (size > 0 && !file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size))) {
        throw std::runtime_error("Failed to write file: " + path);
    }
}

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

int64_t file_size(const std::string& path) {
    struct stat st{};
    if (stat(path.c_str(), &st) != 0) {
        return -1;
    }
    return static_cast<int64_t>(st.st_size);
}

bool file_exists(const std::string& path) {
    struct stat st{};
    return stat(path.c_str(), &st) == 0;
}

} // namespace camcom
