#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace camcom {

// ---------------------------------------------------------------------------
// Binary file I/O helpers
// ---------------------------------------------------------------------------

/// Read the entire contents of a binary file into a byte vector.
/// @throws std::runtime_error on failure.
std::vector<uint8_t> read_binary_file(const std::string& path);

/// Write a byte vector to a binary file (overwrites if it exists).
/// @throws std::runtime_error on failure.
void write_binary_file(const std::string& path, const std::vector<uint8_t>& data);

/// Write a raw byte buffer to a binary file.
/// @throws std::runtime_error on failure.
void write_binary_file(const std::string& path, const uint8_t* data, std::size_t size);

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

/// Return the file size in bytes, or -1 if the file cannot be opened.
int64_t file_size(const std::string& path);

/// Return true if the file exists and is readable.
bool file_exists(const std::string& path);

} // namespace camcom
