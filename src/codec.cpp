#include "codec.hpp"
#include "common.hpp"

#include <algorithm>
#include <cmath>

namespace camcom {

    static inline int color_distance_sq(const cv::Scalar& a, const cv::Scalar& b) {
        int db = static_cast<int>(a[0] - b[0]);
        int dg = static_cast<int>(a[1] - b[1]);
        int dr = static_cast<int>(a[2] - b[2]);
        return db * db + dg * dg + dr * dr;
    }

    void render_frame(cv::Mat& out, const std::vector<uint8_t>& payload, const EncoderConfig& cfg) {
        // Convert payload bytes into 2-bit cells (MSB first within each byte)
        std::vector<int> cells;
        cells.reserve(payload.size() * 4);
        for (uint8_t b : payload) {
            for (int i = 0; i < 4; ++i) {
                int shift = 6 - 2 * i;
                int v = (b >> shift) & 0x3;
                cells.push_back(v);
            }
        }

        const int cells_per_row = cfg.cells_per_row;
        const int total_cells = static_cast<int>(cells.size());
        const int rows = (total_cells + cells_per_row - 1) / cells_per_row;

        const int marker_cells = 4; // marker size in cells
        const int marker_px = marker_cells * cfg.cell_size;
        const int data_w = cells_per_row * cfg.cell_size;
        const int data_h = rows * cfg.cell_size;

        const int img_w = data_w + marker_px * 2;
        const int img_h = data_h + marker_px * 2;

        // Make the frame square by taking the maximum of width and height
        const int square_size = std::max(img_w, img_h);

        out.create(square_size, square_size, CV_8UC3);
        // background gray
        out.setTo(cv::Scalar(128, 128, 128));

        // draw corner finder markers (nested squares) in each corner
        auto draw_finder = [&](int x, int y) {
            cv::Rect r(x, y, marker_px, marker_px);
            // outer black
            cv::rectangle(out, r, cv::Scalar(0, 0, 0), cv::FILLED);
            // inner white
            int inner = cfg.cell_size;
            cv::rectangle(out, cv::Rect(x + inner, y + inner, marker_px - 2 * inner, marker_px - 2 * inner), cv::Scalar(255, 255, 255), cv::FILLED);
            // central black
            int inner2 = inner * 2;
            cv::rectangle(out, cv::Rect(x + inner2, y + inner2, marker_px - 2 * inner2, marker_px - 2 * inner2), cv::Scalar(0, 0, 0), cv::FILLED);
            };

        draw_finder(0, 0);
        draw_finder(square_size - marker_px, 0);
        draw_finder(0, square_size - marker_px);
        draw_finder(square_size - marker_px, square_size - marker_px);

        // draw data cells
        // Calculate center position for data region
        const int origin_x = (square_size - data_w) / 2;
        const int origin_y = (square_size - data_h) / 2;

        for (int idx = 0; idx < total_cells; ++idx) {
            int r = idx / cells_per_row;
            int c = idx % cells_per_row;
            int x = origin_x + c * cfg.cell_size;
            int y = origin_y + r * cfg.cell_size;
            cv::Rect cell_rect(x, y, cfg.cell_size, cfg.cell_size);
            int v = cells[idx];
            cv::rectangle(out, cell_rect, cfg.colors[v], cv::FILLED);
        }

        // draw reference color blocks (inside top margin, near top-left)
        const int ref_cells = cfg.reference_block_size;
        const int ref_px = ref_cells * cfg.cell_size;
        int ref_origin_x = origin_x;
        int ref_origin_y = origin_y - ref_px - cfg.cell_size; // just above data region
        if (ref_origin_y < cfg.cell_size) ref_origin_y = origin_y; // fallback
        for (int k = 0; k < 4; ++k) {
            int rx = ref_origin_x + k * (ref_px + cfg.cell_size / 2);
            cv::Rect r(rx, ref_origin_y, ref_px, ref_px);
            cv::rectangle(out, r, cfg.colors[k], cv::FILLED);
        }
    }

    bool sample_frame(const cv::Mat& warped, std::vector<uint8_t>& out_payload, const EncoderConfig& cfg) {
        // Expect warped image that tightly contains data region including markers.
        // We will infer marker size in pixels by locating the black/white finder at corners.
        // For simplicity assume markers are present and data region starts after marker_px.

        // Try to infer marker_px by searching from corners for large black square.
        const int img_w = warped.cols;
        const int img_h = warped.rows;

        // estimate marker size as 4 * cell_size
        const int marker_px = 4 * cfg.cell_size;
        const int origin_x = marker_px;
        const int origin_y = marker_px;

        const int data_w = img_w - marker_px * 2;
        const int data_h = img_h - marker_px * 2;

        if (data_w <= 0 || data_h <= 0) return false;

        const int cells_per_row = cfg.cells_per_row;
        const int cols = cells_per_row;
        const int rows = data_h / cfg.cell_size;

        if (rows <= 0) return false;

        std::vector<int> cells;
        cells.reserve(rows * cols);

        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                int x = origin_x + c * cfg.cell_size;
                int y = origin_y + r * cfg.cell_size;
                cv::Rect cell_rect(x, y, cfg.cell_size, cfg.cell_size);
                cv::Mat roi = warped(cell_rect);
                cv::Scalar mean = cv::mean(roi);
                // find nearest color
                int best = 0;
                int bestd = color_distance_sq(mean, cfg.colors[0]);
                for (int k = 1; k < 4; ++k) {
                    int d = color_distance_sq(mean, cfg.colors[k]);
                    if (d < bestd) { best = k; bestd = d; }
                }
                cells.push_back(best);
            }
        }

        // pack cells into bytes (4 cells per byte, MSB-first)
        const int total_cells = static_cast<int>(cells.size());
        const int total_bytes = (total_cells / 4);
        out_payload.clear();
        out_payload.reserve(total_bytes);
        for (int i = 0; i < total_bytes; ++i) {
            uint8_t b = 0;
            for (int j = 0; j < 4; ++j) {
                int cell_idx = i * 4 + j;
                int v = cells[cell_idx] & 0x3;
                int shift = 6 - 2 * j;
                b |= static_cast<uint8_t>(v << shift);
            }
            out_payload.push_back(b);
        }

        return true;
    }

    double laplacian_variance(const cv::Mat& img) {
        cv::Mat gray;
        if (img.channels() == 3) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        else gray = img;
        cv::Mat lap;
        cv::Laplacian(gray, lap, CV_64F);
        cv::Scalar mu, sigma;
        cv::meanStdDev(lap, mu, sigma);
        double var = sigma[0] * sigma[0];
        return var;
    }

    cv::Vec3d compute_color_scale(const std::array<cv::Scalar, 4>& expected, const std::array<cv::Scalar, 4>& observed) {
        // compute per-channel scale as average of observed/expected per color, avoid division by zero
        double sb = 0, sg = 0, sr = 0; int cnt = 0;
        for (int i = 0; i < 4; ++i) {
            const cv::Scalar& e = expected[i];
            const cv::Scalar& o = observed[i];
            if (e[0] > 1e-6) { sb += o[0] / e[0]; }
            if (e[1] > 1e-6) { sg += o[1] / e[1]; }
            if (e[2] > 1e-6) { sr += o[2] / e[2]; }
            ++cnt;
        }
        if (cnt == 0) return cv::Vec3d(1, 1, 1);
        return cv::Vec3d(sb / cnt, sg / cnt, sr / cnt);
    }

} // namespace camcom
