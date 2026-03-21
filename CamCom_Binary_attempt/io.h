#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

// 二进制文件I/O辅助函数
// 将二进制文件全部内容读入字节数组
// 失败时抛出std::runtime_error异常
std::vector<uint8_t> read_binary_file(const std::string& path);

// 将字节数组写入二进制文件（如果存在则覆盖）
// 失败时抛出std::runtime_error异常
void write_binary_file(const std::string& path, const std::vector<uint8_t>& data);

// 将原始字节缓冲区写入二进制文件
// 失败时抛出std::runtime_error异常
void write_binary_file(const std::string& path, const uint8_t* data, std::size_t size);

// 工具辅助函数
// 返回文件大小（字节），如果文件无法打开或不是常规文件则返回-1
int64_t file_size(const std::string& path);

// 如果路径存在且为常规文件（非目录）则返回true
bool file_exists(const std::string& path);