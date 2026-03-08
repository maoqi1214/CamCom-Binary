
#include "common.hpp"
#include "io.hpp"
#include "codec.hpp"
#include "rs.hpp"
#include "tracker.hpp"

#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <algorithm>

using namespace camcom;

static void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " <recorded.mp4> <output.bin> <validity_mask.bin>\n"
        << "\n"
        << "  recorded.mp4      Path to the video file captured by the camera.\n"
        << "  output.bin        Path where the decoded binary data will be written.\n"
        << "  validity_mask.bin Path where the per-frame validity mask will be written.\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " captured.mp4 recovered.bin mask.bin\n";
}

static uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

// Improved finder detection using contour hierarchy (nested markers) and Kalman tracking.
static std::optional<cv::Mat> find_and_warp(const cv::Mat& frame, const EncoderConfig& cfg, QuadTracker& tracker) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bw;
    cv::adaptiveThreshold(gray, bw, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 15, 2);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    struct Candidate { cv::Point2f center; double area; };
    std::vector<Candidate> markers;

    for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area < 500) continue;
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true)*0.02, true);
        if (approx.size() != 4) continue;

        // check for nested structure using hierarchy: has child and grandchild
        int child = hierarchy[i][2];
        if (child < 0) continue;
        int grand = hierarchy[child][2];
        if (grand < 0) continue;

        cv::Moments m = cv::moments(approx);
        if (m.m00 == 0) continue;
        markers.push_back({cv::Point2f(static_cast<float>(m.m10/m.m00), static_cast<float>(m.m01/m.m00)), area});
    }

    if (markers.size() < 4) {
        // fallback: if tracker initialized, use predicted corners
        if (tracker.is_initialized()) {
            auto pred = tracker.get();
            std::vector<cv::Point2f> src(pred.begin(), pred.end());
            const int marker_px = 4 * cfg.cell_size;
            const int data_w = cfg.cells_per_row * cfg.cell_size;
            int dst_w = data_w + marker_px*2;
            int dst_h = cfg.cell_size*8 + marker_px*2;
            std::vector<cv::Point2f> dst = { {0,0}, {static_cast<float>(dst_w-1),0}, {static_cast<float>(dst_w-1), static_cast<float>(dst_h-1)}, {0, static_cast<float>(dst_h-1)} };
            cv::Mat H = cv::getPerspectiveTransform(src, dst);
            cv::Mat warped;
            cv::warpPerspective(frame, warped, H, cv::Size(dst_w, dst_h));
            return warped;
        }
        return std::nullopt;
    }

    // choose 4 largest markers
    std::sort(markers.begin(), markers.end(), [](const Candidate&a,const Candidate&b){return a.area>b.area;});
    std::array<cv::Point2f,4> corners;
    for (int i = 0; i < 4; ++i) corners[i] = markers[i].center;

    // sort into TL, TR, BR, BL
    std::sort(corners.begin(), corners.end(), [](const cv::Point2f&a,const cv::Point2f&b){ return a.y < b.y || (a.y==b.y && a.x < b.x);} );
    cv::Point2f tl = corners[0];
    cv::Point2f tr = corners[1];
    cv::Point2f bl = corners[2];
    cv::Point2f br = corners[3];
    if (tl.x > tr.x) std::swap(tl,tr);
    if (bl.x > br.x) std::swap(bl,br);

    std::array<cv::Point2f,4> detected = { tl, tr, br, bl };
    tracker.update(detected);
    auto smoothed = tracker.get();

    const int marker_px = 4 * cfg.cell_size;
    const int data_w = cfg.cells_per_row * cfg.cell_size;
    int dst_w = data_w + marker_px*2;
    int dst_h = cfg.cell_size*8 + marker_px*2;
    std::vector<cv::Point2f> src = { smoothed[0], smoothed[1], smoothed[2], smoothed[3] };
    std::vector<cv::Point2f> dst = { {0,0}, {static_cast<float>(dst_w-1),0}, {static_cast<float>(dst_w-1), static_cast<float>(dst_h-1)}, {0, static_cast<float>(dst_h-1)} };
    cv::Mat H = cv::getPerspectiveTransform(src, dst);
    cv::Mat warped;
    cv::warpPerspective(frame, warped, H, cv::Size(dst_w, dst_h));
    return warped;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        print_usage(argv[0]);
        return static_cast<int>(ExitCode::BadArgs);
    }

    const std::string video_path  = argv[1];
    const std::string output_path = argv[2];
    const std::string mask_path   = argv[3];

    if (!file_exists(video_path)) {
        std::cerr << "Error: video file not found: " << video_path << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    EncoderConfig cfg;

    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: failed to open video: " << video_path << "\n";
        return static_cast<int>(ExitCode::DecodingError);
    }

    std::vector<std::vector<uint8_t>> frames_buffer;
    std::vector<uint8_t> frame_valid; // 1 valid, 0 invalid
    uint32_t expected_total_frames = 0;
    bool have_bootstrap = false;
    bool have_stream_header = false;

    uint32_t stream_rs_nsym = 0;


    cv::Mat frame;
    QuadTracker tracker;
    while (cap.read(frame)) {
        // blur check
        double var = laplacian_variance(frame);
        if (var < 20.0) {
            // skip very blurry frames
            continue;
        }
        auto warped_opt = find_and_warp(frame, cfg, tracker);
        if (!warped_opt) continue;
        cv::Mat warped = *warped_opt;

        // perform color calibration using reference blocks rendered by encoder
        // sample reference blocks (assume they're located just above data region)
        std::array<cv::Scalar,4> observed_refs;
        bool got_refs = true;
        const int marker_px = 4 * cfg.cell_size;
        const int ref_px = cfg.reference_block_size * cfg.cell_size;
        int ref_origin_x = marker_px;
        int ref_origin_y = marker_px - ref_px - cfg.cell_size;
        if (ref_origin_y < 0) ref_origin_y = marker_px; // fallback
        for (int k = 0; k < 4; ++k) {
            int rx = ref_origin_x + k * (ref_px + cfg.cell_size/2);
            cv::Rect r(rx, ref_origin_y, ref_px, ref_px);
            if (r.x < 0 || r.y < 0 || r.x + r.width > warped.cols || r.y + r.height > warped.rows) { got_refs = false; break; }
            observed_refs[k] = cv::mean(warped(r));
        }

        std::vector<uint8_t> sample;
        if (!sample_frame(warped, sample, cfg)) continue;

        // if we have refs, compute color scale and adjust cfg.colors copy
        EncoderConfig cfg_adj = cfg;
        if (got_refs) {
            std::array<cv::Scalar,4> expected;
            for (int k = 0; k < 4; ++k) expected[k] = cfg.colors[k];
            cv::Vec3d scale = compute_color_scale(expected, observed_refs);
            for (int k = 0; k < 4; ++k) {
                cv::Scalar c = cfg.colors[k];
                c[0] = std::min(255.0, c[0] * (1.0/scale[0]));
                c[1] = std::min(255.0, c[1] * (1.0/scale[1]));
                c[2] = std::min(255.0, c[2] * (1.0/scale[2]));
                cfg_adj.colors[k] = c;
            }
            // re-sample using adjusted colors
            if (!sample_frame(warped, sample, cfg_adj)) continue;
        }

        // If we haven't seen any bootstrap, check whether this frame is the bootstrap (unprotected)
        if (!have_bootstrap) {
            // minimal size for bootstrap: u32 magic + u8 version + 5*u32 = 4+1+20 = 25
            if (sample.size() >= 28) {
                const uint8_t* p = sample.data();
                uint32_t magic = read_u32_le(p); p += 4;
                uint8_t version = *p; p += 1;
                if (magic == MAGIC && version == FORMAT_VERSION) {
                    // parse bootstrap fields in same order as encoder
                    uint32_t rs_nsym = read_u32_le(p); p += 4;
                    uint32_t cell_size = read_u32_le(p); p += 4;
                    uint32_t cells_per_row = read_u32_le(p); p += 4;
                    uint32_t payload_per = read_u32_le(p); p += 4;
                    uint32_t fps = read_u32_le(p); p += 4;
                    uint32_t reference_block_size = read_u32_le(p); p += 4;

                    // apply to cfg
                    cfg.cell_size = static_cast<int>(cell_size);
                    cfg.fps = static_cast<int>(fps);
                    cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
                    cfg.cells_per_row = static_cast<int>(cells_per_row);
                    cfg.rs_nsym = static_cast<int>(rs_nsym);
                    cfg.reference_block_size = static_cast<int>(reference_block_size);
                    stream_rs_nsym = rs_nsym;
                    have_bootstrap = true;
                    frame_valid.push_back(1);
                    continue;
                }
            }
            // not bootstrap: skip
            frame_valid.push_back(0);
            continue;
        }

        // If we have bootstrap but not yet stream header, then this frame should be RS-protected StreamHeader
        if (have_bootstrap && !have_stream_header) {
            std::vector<uint8_t> codeword = sample;
            if (stream_rs_nsym > 0) {
                if (!rs::decode(codeword, static_cast<int>(stream_rs_nsym))) {
                    frame_valid.push_back(0);
                    continue;
                }
            }
            const uint8_t* p = codeword.data();
            uint32_t magic = read_u32_le(p); p += 4;
            uint8_t version = *p; p += 1;
            if (!(magic == MAGIC && version == FORMAT_VERSION)) {
                frame_valid.push_back(0);
                continue;
            }
            // parse StreamHeader fields
            uint64_t total_data = 0;
            for (int i = 0; i < 8; ++i) total_data |= static_cast<uint64_t>(p[i]) << (8*i);
            p += 8;
            uint8_t encoding = *p; p += 1;
            uint32_t fps = read_u32_le(p); p += 4;
            uint32_t cell_size = read_u32_le(p); p += 4;
            uint32_t rs_nsym = read_u32_le(p); p += 4;
            uint32_t payload_per = read_u32_le(p); p += 4;
            uint32_t cells_per_row = read_u32_le(p); p += 4;
            uint32_t total_frames_hdr = read_u32_le(p); p += 4;

            // apply to cfg
            cfg.cell_size = static_cast<int>(cell_size);
            cfg.fps = static_cast<int>(fps);
            cfg.payload_bytes_per_frame = static_cast<int>(payload_per);
            cfg.cells_per_row = static_cast<int>(cells_per_row);
            cfg.rs_nsym = static_cast<int>(rs_nsym);
            stream_rs_nsym = rs_nsym;
            expected_total_frames = total_frames_hdr;
            frames_buffer.resize(expected_total_frames);
            have_stream_header = true;
            frame_valid.push_back(1);
            continue;
        }

        // For data frames, sample contains codeword = message + parity
        std::vector<uint8_t> codeword = sample;
        if (stream_rs_nsym > 0) {
            if (!rs::decode(codeword, static_cast<int>(stream_rs_nsym))) {
                frame_valid.push_back(0);
                continue;
            }
        }

        // after RS decode, parse FrameHeader then payload
        const uint8_t* p = codeword.data();
        FrameHeader hdr;
        hdr.magic = read_u32_le(p); p += 4;
        hdr.version = *p; p += 1;
        hdr.frame_index = read_u32_le(p); p += 4;
        hdr.total_frames = read_u32_le(p); p += 4;
        hdr.payload_bytes = read_u32_le(p); p += 4;
        hdr.checksum = read_u32_le(p); p += 4;

        if (hdr.magic != MAGIC || hdr.version != FORMAT_VERSION) {
            frame_valid.push_back(0);
            continue;
        }

        size_t payload_len = hdr.payload_bytes;
        if (p + payload_len > codeword.data() + codeword.size()) {
            frame_valid.push_back(0);
            continue;
        }

        const uint8_t* payload_ptr = p;
        const uint32_t calc = crc32(payload_ptr, payload_len);
        if (calc != hdr.checksum) {
            frame_valid.push_back(0);
            if (hdr.total_frames > 0) {
                if (frames_buffer.size() < hdr.total_frames) frames_buffer.resize(hdr.total_frames);
                if (hdr.frame_index < frames_buffer.size()) frames_buffer[hdr.frame_index].clear();
            }
            continue;
        }

        // accept
        if (hdr.total_frames > 0 && expected_total_frames == 0) {
            expected_total_frames = hdr.total_frames;
            frames_buffer.resize(expected_total_frames);
        }

        if (hdr.frame_index < frames_buffer.size()) {
            frames_buffer[hdr.frame_index] = std::vector<uint8_t>(payload_ptr, payload_ptr + payload_len);
            frame_valid.push_back(1);
        } else {
            frame_valid.push_back(0);
        }
    }

    // reassemble
    std::vector<uint8_t> recovered;
    if (expected_total_frames == 0) {
        std::cerr << "Warning: no frames decoded successfully." << std::endl;
    } else {
        for (uint32_t i = 0; i < expected_total_frames; ++i) {
            if (i < frames_buffer.size() && !frames_buffer[i].empty()) {
                recovered.insert(recovered.end(), frames_buffer[i].begin(), frames_buffer[i].end());
            } else {
                // missing frame -> skip (could insert zeros)
            }
        }
    }

    // write outputs
    try {
        write_binary_file(output_path, recovered);
        write_binary_file(mask_path, frame_valid);
    } catch (const std::exception& ex) {
        std::cerr << "Error writing output: " << ex.what() << "\n";
        return static_cast<int>(ExitCode::IoError);
    }

    std::cout << "[decoder] Done. Recovered bytes=" << recovered.size() << "\n";
    return static_cast<int>(ExitCode::Ok);
}
