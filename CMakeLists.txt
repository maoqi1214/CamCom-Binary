cmake_minimum_required(VERSION 3.16)
project(CamComBinary VERSION 1.0.0 LANGUAGES CXX)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Place compiled binaries in build/bin for convenience
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ---------------------------------------------------------------------------
# Find OpenCV
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "OpenCV not found. Install it and re-run CMake.")
endif()

# ---------------------------------------------------------------------------
# Find FFmpeg via pkg-config (libavcodec, libavformat, libavutil)
# ---------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED
    libavcodec
    libavformat
    libavutil
)
if(FFMPEG_FOUND)
    message(STATUS "FFmpeg found: avcodec ${FFMPEG_libavcodec_VERSION}")
else()
    message(FATAL_ERROR "FFmpeg not found. Install libavcodec/libavformat/libavutil and re-run CMake.")
endif()

# ---------------------------------------------------------------------------
# Common include paths
# ---------------------------------------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS}
)

# ---------------------------------------------------------------------------
# encoder executable
# ---------------------------------------------------------------------------
add_executable(encoder
    src/encoder.cpp
)
target_link_libraries(encoder
    PRIVATE
        ${OpenCV_LIBS}
        ${FFMPEG_LIBRARIES}
)
target_compile_options(encoder PRIVATE ${FFMPEG_CFLAGS_OTHER})

# ---------------------------------------------------------------------------
# decoder executable
# ---------------------------------------------------------------------------
add_executable(decoder
    src/decoder.cpp
)
target_link_libraries(decoder
    PRIVATE
        ${OpenCV_LIBS}
        ${FFMPEG_LIBRARIES}
)
target_compile_options(decoder PRIVATE ${FFMPEG_CFLAGS_OTHER})

# ---------------------------------------------------------------------------
# Install targets (optional)
# ---------------------------------------------------------------------------
install(TARGETS encoder decoder DESTINATION bin)
