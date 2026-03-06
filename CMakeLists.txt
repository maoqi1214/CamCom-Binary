cmake_minimum_required(VERSION 3.10)

project(CamComBinary)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(include)
add_executable(camcom src/common.cpp src/io.cpp src/encoder.cpp src/decoder.cpp)