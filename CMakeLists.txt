cmake_minimum_required(VERSION 3.16)

project(CamComBinary
  VERSION 0.1.0
  DESCRIPTION "Screen-to-camera binary transfer"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W4 /utf-8)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)

add_library(camcom_core
  src/codec.cpp
  src/common.cpp
  src/ffmpeg.cpp
  src/io.cpp
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

add_executable(encode src/encoder.cpp)
target_link_libraries(encode PRIVATE camcom_core)

add_executable(decode src/decoder.cpp)
target_link_libraries(decode PRIVATE camcom_core)

set_target_properties(encode decode PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
