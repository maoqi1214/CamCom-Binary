#include "common.hpp"
#include "io.hpp"
#include "codec.hpp"
#include "rs.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <clocale>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

using namespace camcom;
namespace fs = std::filesystem;

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <input.bin> <output.mp4> <fps>\n"
        << "\n"
        << "  input.bin    Path to the binary file to encode.\n"
        << "  output.mp4   Path to the output video file.\n"
        << "  fps          Frames per second (<=15 recommended).\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " payload.bin out.mp4 10\n";
}

static void init_console_utf8() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    std::setlocale(LC_ALL, ".UTF-8");
}

static void print_next_steps_guide(const std::string& output_path) {
    std::cout
        << "\n==================== 操作指引====================\n"
        << "0) Visual Studio 选择 x64-Release 配置\n"
        << "   - PowerShell 先进入可执行文件目录:\n"
        << "     cd D:CamCom-Binary\\out\\build\\x64-Release\\bin\n"
        << "1) 先准备输入文件\n"
        << "   - 把要传输的二进制文件放到项目目录（例如 tests/sample_input.bin）\n"
        << "\n"
        << "2) 运行 encoder（本程序）\n"
        << "   - 命令格式: .\\encoder.exe <input.bin> <output.mp4> <fps>\n"
        << "   - 示例命令: .\\encoder.exe ..\\..\\..\\..\\tests\\sample_input.bin out.mp4 15\n"
        << "   - 说明: fps 建议 <= 15，过高会降低实拍解码成功率\n"
        << "\n"
        << "3) 编码完成后会得到视频文件\n"
        << "   - 当前输出视频: " << output_path << "\n"
        << "\n"
        << "4) 运行 decoder 还原文件\n"
        << "   - 命令格式: .\\decoder.exe <input.mp4> <output.bin> [reference_input.bin]\n"
        << "   - 示例命令: .\\decoder.exe " << output_path << " recovered.bin ..\\..\\..\\..\\tests\\sample_input.bin\n"
        << "   - 第3个参数可选: 用于和原始输入做正确性对比\n"
        << "\n"
        << "5) 结果检查\n"
        << "   - 看 recovered.bin 是否生成\n"
        << "   - 若传入 reference_input.bin，查看 decoder 输出的对比信息\n"
        << "\n"
        << "常见报错\n"
        << "   - ffmpeg command failed: 需要安装 ffmpeg 并加入 PATH\n"
        << "============================================================\n\n";
}

static void serialize_u8(std::vector<uint8_t>& buf, uint8_t v) { buf.push_back(v); }
static void serialize_u32(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFFu));
    buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFFu));
}
static void serialize_u64(std::vector<uint8_t>& buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) buf.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
}

static std::vector<uint8_t> build_bootstrap(const EncoderConfig& cfg) {
    std::vector<uint8_t> buf;
    serialize_u32(buf, MAGIC);
    serialize_u8(buf, FORMAT_VERSION);
    serialize_u32(buf, static_cast<uint32_t>(cfg.rs_nsym));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(buf, static_cast<uint32_t>(cfg.reference_block_size));
    return buf;
}

static std::vector<uint8_t> build_stream_header(const EncoderConfig& cfg, size_t total_bytes, uint32_t total_frames) {
    std::vector<uint8_t> buf;
    serialize_u32(buf, MAGIC);
    serialize_u8(buf, FORMAT_VERSION);
    serialize_u64(buf, static_cast<uint64_t>(total_bytes));
    serialize_u8(buf, static_cast<uint8_t>(Encoding::Binary));
    serialize_u32(buf, static_cast<uint32_t>(cfg.fps));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cell_size));
    serialize_u32(buf, static_cast<uint32_t>(cfg.rs_nsym));
    serialize_u32(buf, static_cast<uint32_t>(cfg.payload_bytes_per_frame));
    serialize_u32(buf, static_cast<uint32_t>(cfg.cells_per_row));
    serialize_u32(buf, total_frames);

    std::vector<uint8_t> parity = rs::encode(buf, cfg.rs_nsym);
    buf.insert(buf.end(), parity.begin(), parity.end());
    return buf;
}

static std::vector<uint8_t> build_frame_codeword(uint32_t fi, uint32_t total_frames, const std::vector<uint8_t>& data, size_t offset, size_t chunk, const EncoderConfig& cfg) {
    std::vector<uint8_t> frame_buf;
    serialize_u32(frame_buf, MAGIC);
    frame_buf.push_back(FORMAT_VERSION);
    serialize_u32(frame_buf, fi);
    serialize_u32(frame_buf, total_frames);
    serialize_u32(frame_buf, static_cast<uint32_t>(chunk));
    const size_t checksum_pos = frame_buf.size();
    serialize_u32(frame_buf, 0);

    frame_buf.insert(frame_buf.end(), data.begin() + offset, data.begin() + offset + chunk);

    const uint32_t checksum = crc32(frame_buf.data() + checksum_pos + 4, chunk);
    frame_buf[checksum_pos + 0] = static_cast<uint8_t>(checksum & 0xFFu);
    frame_buf[checksum_pos + 1] = static_cast<uint8_t>((checksum >> 8) & 0xFFu);
    frame_buf[checksum_pos + 2] = static_cast<uint8_t>((checksum >> 16) & 0xFFu);
    frame_buf[checksum_pos + 3] = static_cast<uint8_t>((checksum >> 24) & 0xFFu);

    std::vector<uint8_t> parity = rs::encode(frame_buf, cfg.rs_nsym);
    frame_buf.insert(frame_buf.end(), parity.begin(), parity.end());
    return frame_buf;
}

static void write_frame_pair(const cv::Mat& img, const cv::Mat& black, const std::string& temp_dir, size_t& frame_count) {
    const std::string frame_path = temp_dir + "/frame_" + std::to_string(frame_count++) + ".png";
    cv::imwrite(frame_path, img);
    const std::string black_path = temp_dir + "/frame_" + std::to_string(frame_count++) + ".png";
    cv::imwrite(black_path, black);
}

static cv::Mat fit_to_video_canvas(const cv::Mat& src, int canvas_w, int canvas_h) {
    cv::Mat dst(canvas_h, canvas_w, CV_8UC3, cv::Scalar(128, 128, 128));
    const int w = std::min(src.cols, canvas_w);
    const int h = std::min(src.rows, canvas_h);
    src(cv::Rect(0, 0, w, h)).copyTo(dst(cv::Rect(0, 0, w, h)));
    return dst;
}

int main(int argc, char* argv[]) {
    init_console_utf8();
    std::cout << "[encoder][步骤 0/8] 启动编码器\n";
    if (argc != 4) {
        print_usage(argv[0]);
        std::cout << "\n[encoder] 参数不完整，按上面的 Usage 执行。\n";
        print_next_steps_guide("out.mp4");
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string input_path = argv[1];
    const std::string output_path = argv[2];
    const int fps = std::stoi(argv[3]);

    if (fps <= 0 || fps > 60) {
        std::cerr << "Error: fps must be a positive integer (reasonable <=60).\n";
        std::cerr << "Tip: 建议设置为 10~15，实拍更稳定。\n";
        return static_cast<int>(ExitCode::BadArgs);
    }

    if (!file_exists(input_path)) {
        std::cerr << "Error: input file not found: " << input_path << "\n";
        std::cerr << "Tip: 请确认 input.bin 路径正确，建议传绝对路径。\n";
        return static_cast<int>(ExitCode::IoError);
    }

    std::cout << "[encoder] Input : " << input_path << "\n"
        << "[encoder] Output: " << output_path << "\n"
        << "[encoder] FPS   : " << fps << "\n";

    std::cout << "[encoder][步骤 1/8] 读取输入二进制文件...\n";
    const auto data = read_binary_file(input_path);
    std::cout << "[encoder] 输入文件字节数: " << data.size() << "\n";

    std::cout << "[encoder][步骤 2/8] 设置编码参数(cell_size/cells_per_row/payload) ...\n";
    EncoderConfig cfg;
    cfg.fps = fps;
    // 数据网格目标：每行 108 个点。
    // 将 cell_size 提升到 8，以提高在 H.264 YUV420p 色度子采样下的稳定性。
    cfg.cell_size = 8;
    cfg.cells_per_row = 108;
    cfg.payload_bytes_per_frame = 2875;

    const size_t payload_per_frame = static_cast<size_t>(cfg.payload_bytes_per_frame);
    const uint32_t total_frames = static_cast<uint32_t>((data.size() + payload_per_frame - 1) / payload_per_frame);

    const int frame_header_bytes = 4 + 1 + 4 + 4 + 4 + 4;
    const int max_codeword_bytes = frame_header_bytes + cfg.payload_bytes_per_frame + cfg.rs_nsym;
    const int max_cells = max_codeword_bytes * 4;
    const int max_rows = static_cast<int>((max_cells + cfg.cells_per_row - 1) / cfg.cells_per_row);
    const int video_w = (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS) * cfg.cell_size;
    const int video_h = (max_rows + 2 * FINDER_MARKER_CELLS) * cfg.cell_size;

    std::cout << "[encoder] total bytes=" << data.size() << " frames=" << total_frames << "\n";
    std::cout << "[encoder] 视频画布: " << video_w << "x" << video_h << "\n";

    cv::Mat first_img;

    // 创建临时帧目录
    std::cout << "[encoder][步骤 3/8] 创建临时帧目录 temp_frames ...\n";
    const std::string temp_dir = "temp_frames";
    if (fs::exists(temp_dir)) {
        fs::remove_all(temp_dir);
    }
    fs::create_directory(temp_dir);

    size_t frame_count = 0;

    // 写入小型 Bootstrap 帧（不加保护），携带解码所需参数
    std::cout << "[encoder][步骤 4/8] 写入 Bootstrap 帧...\n";
    std::vector<uint8_t> bootstrap_buf = build_bootstrap(cfg);

    render_frame(first_img, bootstrap_buf, cfg);
    first_img = fit_to_video_canvas(first_img, video_w, video_h);
    cv::Mat black0(video_h, video_w, CV_8UC3, cv::Scalar(0, 0, 0));

    // 重复输出若干次 Bootstrap 帧，提升鲁棒性
    const int BOOTSTRAP_REPEAT = 3;
    for (int i = 0; i < BOOTSTRAP_REPEAT; ++i) {
        write_frame_pair(first_img, black0, temp_dir, frame_count);
    }

    // 写入带 RS 冗余保护的 StreamHeader
    std::cout << "[encoder][步骤 5/8] 写入 StreamHeader 帧...\n";
    std::vector<uint8_t> stream_buf = build_stream_header(cfg, data.size(), total_frames);
    render_frame(first_img, stream_buf, cfg);
    first_img = fit_to_video_canvas(first_img, video_w, video_h);
    // 多次重复输出受 RS 保护的 StreamHeader，确保解码端更容易捕获
    const int STREAMHDR_REPEAT = 3;
    for (int i = 0; i < STREAMHDR_REPEAT; ++i) {
        write_frame_pair(first_img, black0, temp_dir, frame_count);
    }

    std::cout << "[encoder][步骤 6/8] 生成数据帧与黑帧...\n";
    for (uint32_t fi = 0; fi < total_frames; ++fi) {
        const size_t offset = static_cast<size_t>(fi) * payload_per_frame;
        const size_t remain = (offset < data.size()) ? (data.size() - offset) : 0;
        const size_t chunk = std::min(remain, payload_per_frame);

        std::vector<uint8_t> frame_buf = build_frame_codeword(fi, total_frames, data, offset, chunk, cfg);

        // 渲染数据帧图像
        render_frame(first_img, frame_buf, cfg);
        first_img = fit_to_video_canvas(first_img, video_w, video_h);
        const std::string frame_path = temp_dir + "/frame_" + std::to_string(frame_count++) + ".png";
        cv::imwrite(frame_path, first_img);

        // 插入黑色过渡帧，降低运动模糊伪影影响
        cv::Mat black(video_h, video_w, CV_8UC3, cv::Scalar(0, 0, 0));
        const std::string black_path = temp_dir + "/frame_" + std::to_string(frame_count++) + ".png";
        cv::imwrite(black_path, black);

        if ((fi + 1) % 20 == 0 || fi + 1 == total_frames) {
            std::cout << "[encoder] 已完成数据帧 " << (fi + 1) << "/" << total_frames << "\n";
        }
    }

    // 使用 ffmpeg 将图像序列合成为视频
    std::cout << "[encoder][步骤 7/8] 调用 ffmpeg 合成视频...\n";
    std::string ffmpeg_cmd =
        "ffmpeg -y -framerate " + std::to_string(fps) +
        " -i " + temp_dir +
        "/frame_%d.png -c:v libx264 -crf 12 -preset slow -pix_fmt yuv420p " +
        output_path;
    std::cout << "[encoder] Running ffmpeg command: " << ffmpeg_cmd << "\n";
    int ffmpeg_result = system(ffmpeg_cmd.c_str());
    if (ffmpeg_result != 0) {
        std::cerr << "Error: ffmpeg command failed with exit code " << ffmpeg_result << "\n";
        // 清理临时文件
        fs::remove_all(temp_dir);
        return static_cast<int>(ExitCode::EncodingError);
    }

    // 清理临时文件
    std::cout << "[encoder][步骤 8/8] 清理临时文件...\n";
    fs::remove_all(temp_dir);

    std::cout << "[encoder] Done.\n";
    print_next_steps_guide(output_path);
    return static_cast<int>(ExitCode::Ok);
}