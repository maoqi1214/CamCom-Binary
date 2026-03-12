cmake_minimum_required(VERSION 3.16)

project(CamComBinary
  VERSION 0.1.0
  DESCRIPTION "屏幕到相机的二进制传输系统"
  LANGUAGES CXX
)

# ===== C++ 标准配置 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ===== 编译告警等级 =====
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 第三方依赖：OpenCV =====
# 说明：本项目运行时通过命令行调用 ffmpeg 可执行文件，不直接链接 FFmpeg 开发库。
# 因此这里仅保留 OpenCV 作为编译期依赖，移除对本地硬编码 FFmpeg 库变量的依赖。
find_package(OpenCV REQUIRED)

# ===== 核心库 =====
add_library(camcom_core
  src/common.cpp
  src/io.cpp
  src/codec.cpp
  src/rs.cpp
  src/tracker.cpp
)

target_include_directories(camcom_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camcom_core
  PUBLIC
    ${OpenCV_LIBS}
)

# ===== 可执行文件：encoder =====
add_executable(encoder src/encoder.cpp)
target_link_libraries(encoder PRIVATE camcom_core)

# ===== 可执行文件：decoder =====
add_executable(decoder src/decoder.cpp)
target_link_libraries(decoder PRIVATE camcom_core)

# ===== 测试程序 =====
add_executable(rs_unit_test tests/rs_unit_test.cpp)
target_link_libraries(rs_unit_test PRIVATE camcom_core)

add_executable(integration_test tests/integration_test.cpp)
target_link_libraries(integration_test PRIVATE camcom_core)

add_executable(integration_metrics tests/integration_metrics.cpp)
target_link_libraries(integration_metrics PRIVATE camcom_core)

# ===== 输出目录 =====
# 将主要可执行文件统一输出到 build/bin 目录
set_target_properties(encoder decoder PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
