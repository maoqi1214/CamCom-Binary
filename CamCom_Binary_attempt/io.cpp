#include "io.h"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

// ---------------------------------------------------------------------------
// Binary file I/O
// ---------------------------------------------------------------------------

std::vector<uint8_t> read_binary_file(const std::string& path) {
    // Use binary mode and ate (at end) to get size immediately
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

    // Only attempt read if size > 0 (reading 0 bytes is valid but some streams behave oddly)
    if (size > 0) {
        if (!file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(size))) {
            // Check if we failed due to EOF before reading enough bytes
            if (!file.eof()) {
                throw std::runtime_error("Failed to read file content: " + path);
            }
            // If EOF reached early, resize buffer to actual read count (optional safety)
            buffer.resize(static_cast<std::size_t>(file.gcount()));
        }
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

    if (size > 0) {
        if (!file.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size))) {
            throw std::runtime_error("Failed to write file content: " + path);
        }
    }
    // Explicitly close to ensure flush and error check (destructor does this too, but explicit is clearer)
    file.close();
    if (file.fail()) {
        throw std::runtime_error("Failed to finalize file write: " + path);
    }
}

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

int64_t file_size(const std::string& path) {
    std::error_code error;
    const std::filesystem::path file_path(path);
    if (!std::filesystem::exists(file_path, error) || error) {
        return -1;
    }
    if (!std::filesystem::is_regular_file(file_path, error) || error) {
        return -1;
    }
    const auto size = std::filesystem::file_size(file_path, error);
    if (error) {
        return -1;
    }
    return static_cast<int64_t>(size);
}

bool file_exists(const std::string& path) {
    return file_size(path) >= 0;
}