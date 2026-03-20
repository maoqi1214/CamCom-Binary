// Rendering, perspective correction, and grid sampling for visual frames.
#include "codec.hpp"
#include "common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace camcom {
namespace {

int color_distance_sq(const cv::Scalar& a, const cv::Scalar& b) {
    const int db = static_cast<int>(a[0] - b[0]);
    const int dg = static_cast<int>(a[1] - b[1]);
    const int dr = static_cast<int>(a[2] - b[2]);
    return db * db + dg * dg + dr * dr;
}

double quad_score(
    const cv::Point2f& tl,
    const cv::Point2f& tr,
    const cv::Point2f& bl,
    const cv::Point2f& br) {
    const double top_w = cv::norm(tr - tl);
    const double bottom_w = cv::norm(br - bl);
    const double left_h = cv::norm(bl - tl);
    const double right_h = cv::norm(br - tr);
    const double min_edge = std::min({top_w, bottom_w, left_h, right_h});
    if (min_edge < 1.0) {
        return -1.0;
    }

    const double edge_balance =
        std::abs(top_w - bottom_w) / std::max(top_w, bottom_w) +
        std::abs(left_h - right_h) / std::max(left_h, right_h);

    const cv::Point2f top_mid = (tl + tr) * 0.5f;
    const cv::Point2f bottom_mid = (bl + br) * 0.5f;
    const cv::Point2f left_mid = (tl + bl) * 0.5f;
    const cv::Point2f right_mid = (tr + br) * 0.5f;
    const double orthogonality =
        std::abs((right_mid.x - left_mid.x) * (bottom_mid.x - top_mid.x) +
                 (right_mid.y - left_mid.y) * (bottom_mid.y - top_mid.y)) /
        std::max(1.0, cv::norm(right_mid - left_mid) * cv::norm(bottom_mid - top_mid));

    const double area = std::abs(
        tl.x * tr.y - tl.y * tr.x +
        tr.x * br.y - tr.y * br.x +
        br.x * bl.y - br.y * bl.x +
        bl.x * tl.y - bl.y * tl.x) * 0.5;

    return area - 800.0 * edge_balance - 1200.0 * orthogonality;
}

bool warp_with_finders(const cv::Mat& src, cv::Mat& warped) {
    cv::Mat gray;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    cv::Mat bin;
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    struct Candidate {
        double area;
        cv::Point2f center;
    };

    std::vector<Candidate> candidates;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < 100.0) {
            continue;
        }

        std::vector<cv::Point> poly;
        cv::approxPolyDP(contour, poly, 0.05 * cv::arcLength(contour, true), true);
        if (poly.size() < 4) {
            continue;
        }

        const cv::Rect bounds = cv::boundingRect(poly);
        const double aspect = static_cast<double>(bounds.width) / std::max(1, bounds.height);
        if (aspect < 0.5 || aspect > 2.0) {
            continue;
        }

        const double fill_ratio = area / std::max(1.0, static_cast<double>(bounds.area()));
        if (fill_ratio < 0.30 || fill_ratio > 1.05) {
            continue;
        }

        const cv::Moments moments = cv::moments(poly);
        if (std::abs(moments.m00) < 1e-6) {
            continue;
        }

        candidates.push_back({
            area,
            cv::Point2f(
                static_cast<float>(moments.m10 / moments.m00),
                static_cast<float>(moments.m01 / moments.m00)),
        });
    }

    if (candidates.size() < 4) {
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
        return lhs.area > rhs.area;
    });
    if (candidates.size() > 16) {
        candidates.resize(16);
    }

    const int stat_n = std::min(8, static_cast<int>(candidates.size()));
    std::vector<double> area_stats;
    area_stats.reserve(stat_n);
    for (int i = 0; i < stat_n; ++i) {
        area_stats.push_back(candidates[i].area);
    }
    std::sort(area_stats.begin(), area_stats.end());

    const double median_area = area_stats[stat_n / 2];
    const double min_area = median_area * 0.35;
    const double max_area = median_area * 2.8;
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(), [&](const Candidate& c) {
            return c.area < min_area || c.area > max_area;
        }),
        candidates.end());

    if (candidates.size() < 4) {
        return false;
    }

    bool found_quad = false;
    double best_score = -std::numeric_limits<double>::infinity();
    cv::Point2f tl;
    cv::Point2f tr;
    cv::Point2f bl;
    cv::Point2f br;

    for (int a = 0; a < static_cast<int>(candidates.size()); ++a) {
        for (int b = a + 1; b < static_cast<int>(candidates.size()); ++b) {
            for (int c = b + 1; c < static_cast<int>(candidates.size()); ++c) {
                for (int d = c + 1; d < static_cast<int>(candidates.size()); ++d) {
                    std::array<cv::Point2f, 4> pts = {
                        candidates[a].center,
                        candidates[b].center,
                        candidates[c].center,
                        candidates[d].center,
                    };
                    std::sort(pts.begin(), pts.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
                        return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
                    });

                    const cv::Point2f local_tl = pts[0].x < pts[1].x ? pts[0] : pts[1];
                    const cv::Point2f local_tr = pts[0].x < pts[1].x ? pts[1] : pts[0];
                    const cv::Point2f local_bl = pts[2].x < pts[3].x ? pts[2] : pts[3];
                    const cv::Point2f local_br = pts[2].x < pts[3].x ? pts[3] : pts[2];

                    if (local_tl.x >= local_tr.x || local_bl.x >= local_br.x ||
                        local_tl.y >= local_bl.y || local_tr.y >= local_br.y) {
                        continue;
                    }

                    const double score = quad_score(local_tl, local_tr, local_bl, local_br);
                    if (score > best_score) {
                        best_score = score;
                        tl = local_tl;
                        tr = local_tr;
                        bl = local_bl;
                        br = local_br;
                        found_quad = true;
                    }
                }
            }
        }
    }

    if (!found_quad) {
        return false;
    }

    const double w1 = cv::norm(tr - tl);
    const double w2 = cv::norm(br - bl);
    const double h1 = cv::norm(bl - tl);
    const double h2 = cv::norm(br - tr);
    const double target = std::max({w1, w2, h1, h2});
    if (target < 1.0) {
        return false;
    }

    const int dst_size = static_cast<int>(std::ceil(target));
    std::vector<cv::Point2f> dst = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(dst_size - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(dst_size - 1)),
        cv::Point2f(static_cast<float>(dst_size - 1), static_cast<float>(dst_size - 1)),
    };

    const cv::Mat transform =
        cv::getPerspectiveTransform(std::vector<cv::Point2f>{tl, tr, bl, br}, dst);
    cv::warpPerspective(src, warped, transform, cv::Size(dst_size, dst_size));
    return true;
}

cv::Mat center_crop_square(const cv::Mat& src) {
    const int side = std::min(src.cols, src.rows);
    const int x = (src.cols - side) / 2;
    const int y = (src.rows - side) / 2;
    return src(cv::Rect(x, y, side, side)).clone();
}

} // namespace

void render_frame(cv::Mat& out, const std::vector<uint8_t>& payload, const EncoderConfig& cfg) {
    std::vector<int> cells;
    cells.reserve(payload.size() * 4);
    for (const uint8_t byte : payload) {
        for (int i = 0; i < 4; ++i) {
            const int shift = 6 - 2 * i;
            cells.push_back((byte >> shift) & 0x3);
        }
    }

    const int total_cells = static_cast<int>(cells.size());
    const int rows = (total_cells + cfg.cells_per_row - 1) / cfg.cells_per_row;
    const int marker_px = FINDER_MARKER_CELLS * cfg.cell_size;
    const int data_w = cfg.cells_per_row * cfg.cell_size;
    const int data_h = rows * cfg.cell_size;
    const int img_w = data_w + marker_px * 2;
    const int img_h = data_h + marker_px * 2;

    out.create(img_h, img_w, CV_8UC3);
    out.setTo(cv::Scalar(255, 255, 255));

    auto draw_finder = [&](int x, int y) {
        cv::rectangle(out, cv::Rect(x, y, marker_px, marker_px), cv::Scalar(0, 0, 0), cv::FILLED);
        const int inner = cfg.cell_size;
        cv::rectangle(
            out,
            cv::Rect(x + inner, y + inner, marker_px - 2 * inner, marker_px - 2 * inner),
            cv::Scalar(255, 255, 255),
            cv::FILLED);
        const int inner2 = inner * 2;
        cv::rectangle(
            out,
            cv::Rect(x + inner2, y + inner2, marker_px - 2 * inner2, marker_px - 2 * inner2),
            cv::Scalar(0, 0, 0),
            cv::FILLED);
    };

    draw_finder(0, 0);
    draw_finder(img_w - marker_px, 0);
    draw_finder(0, img_h - marker_px);
    draw_finder(img_w - marker_px, img_h - marker_px);

    const int small_marker_px = 2 * cfg.cell_size;
    auto draw_small = [&](int x, int y) {
        cv::rectangle(out, cv::Rect(x, y, small_marker_px, small_marker_px), cv::Scalar(0, 0, 0), cv::FILLED);
        const int inner = std::max(1, cfg.cell_size / 2);
        cv::rectangle(
            out,
            cv::Rect(x + inner, y + inner, small_marker_px - 2 * inner, small_marker_px - 2 * inner),
            cv::Scalar(255, 255, 255),
            cv::FILLED);
    };

    draw_small((img_w - small_marker_px) / 2, 0);
    draw_small((img_w - small_marker_px) / 2, img_h - small_marker_px);
    draw_small(0, (img_h - small_marker_px) / 2);
    draw_small(img_w - small_marker_px, (img_h - small_marker_px) / 2);

    const int origin_x = marker_px;
    const int origin_y = marker_px;
    for (int index = 0; index < total_cells; ++index) {
        const int row = index / cfg.cells_per_row;
        const int col = index % cfg.cells_per_row;
        cv::rectangle(
            out,
            cv::Rect(origin_x + col * cfg.cell_size, origin_y + row * cfg.cell_size, cfg.cell_size, cfg.cell_size),
            cfg.colors[cells[index]],
            cv::FILLED);
    }

    const int ref_px = cfg.reference_block_size * cfg.cell_size;
    int ref_y = origin_y - ref_px - cfg.cell_size;
    if (ref_y < cfg.cell_size) {
        ref_y = origin_y;
    }
    for (int k = 0; k < 4; ++k) {
        const int ref_x = origin_x + k * (ref_px + cfg.cell_size / 2);
        cv::rectangle(out, cv::Rect(ref_x, ref_y, ref_px, ref_px), cfg.colors[k], cv::FILLED);
    }
}

bool sample_frame(const cv::Mat& frame, std::vector<uint8_t>& out_payload, EncoderConfig& cfg) {
    const bool exact_raw_layout =
        frame.cols == (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS) * cfg.cell_size;

    auto sample_aligned = [&](const cv::Mat& aligned, const char* label) -> bool {
        const int img_w = aligned.cols;
        const int img_h = aligned.rows;

        int origin_x = 0;
        int origin_y = 0;
        double cell_w = cfg.cell_size;
        double cell_h = cfg.cell_size;
        int rows = 0;

        if (std::string(label) == "warped") {
            cell_w = static_cast<double>(img_w) / (cfg.cells_per_row + FINDER_MARKER_CELLS);
            cell_h = cell_w;
            origin_x = static_cast<int>(std::round((FINDER_MARKER_CELLS / 2.0) * cell_w));
            origin_y = static_cast<int>(std::round((FINDER_MARKER_CELLS / 2.0) * cell_h));
            rows = static_cast<int>(std::round(img_h / cell_h)) - FINDER_MARKER_CELLS;
        } else if (std::string(label) == "raw" &&
                   img_w == (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS) * cfg.cell_size) {
            origin_x = FINDER_MARKER_CELLS * cfg.cell_size;
            origin_y = FINDER_MARKER_CELLS * cfg.cell_size;
            rows = (img_h - 2 * FINDER_MARKER_CELLS * cfg.cell_size) / cfg.cell_size;
        } else {
            cv::Mat gray;
            if (aligned.channels() == 3) {
                cv::cvtColor(aligned, gray, cv::COLOR_BGR2GRAY);
            } else {
                gray = aligned;
            }

            std::vector<double> row_std(img_h);
            std::vector<double> col_std(img_w);

            for (int y = 0; y < img_h; ++y) {
                cv::Scalar mean;
                cv::Scalar stddev;
                cv::meanStdDev(gray.row(y), mean, stddev);
                row_std[y] = stddev[0];
            }
            for (int x = 0; x < img_w; ++x) {
                cv::Scalar mean;
                cv::Scalar stddev;
                cv::meanStdDev(gray.col(x), mean, stddev);
                col_std[x] = stddev[0];
            }

            auto find_band = [](const std::vector<double>& values, double frac) {
                double maxv = 0.0;
                for (const double v : values) {
                    maxv = std::max(maxv, v);
                }
                const double threshold = maxv * frac;
                int lo = -1;
                int hi = -1;
                for (int i = 0; i < static_cast<int>(values.size()); ++i) {
                    if (values[i] >= threshold) {
                        if (lo == -1) {
                            lo = i;
                        }
                        hi = i;
                    }
                }
                return std::pair<int, int>(lo, hi);
            };

            const auto [row_lo, row_hi] = find_band(row_std, 0.3);
            const auto [col_lo, col_hi] = find_band(col_std, 0.3);
            (void)col_hi;
            if (row_lo == -1 || col_lo == -1) {
                return false;
            }

            origin_x = std::max(0, col_lo - (col_lo % cfg.cell_size));
            origin_y = std::max(0, row_lo - (row_lo % cfg.cell_size));
            const int data_w = cfg.cells_per_row * cfg.cell_size;
            if (origin_x + data_w > img_w) {
                origin_x = std::max(0, img_w - data_w);
            }
            rows = (row_hi - origin_y + 1) / cfg.cell_size;
        }

        if (rows <= 0) {
            return false;
        }

        std::array<cv::Scalar, 4> classifier_colors = {
            cfg.colors[0], cfg.colors[1], cfg.colors[2], cfg.colors[3],
        };

        const double ref_cells = static_cast<double>(cfg.reference_block_size);
        const double ref_gap = std::max(1.0, cell_w * 0.5);
        const double ref_w = std::max(1.0, ref_cells * cell_w);
        const double ref_h = std::max(1.0, ref_cells * cell_h);
        const double ref_y = origin_y - ref_h - cell_h;
        if (ref_y >= 0.0) {
            std::array<cv::Scalar, 4> observed_refs{};
            bool refs_ok = true;

            for (int k = 0; k < 4; ++k) {
                const double ref_x = origin_x + k * (ref_w + ref_gap);
                const int x0 = static_cast<int>(std::round(ref_x));
                const int y0 = static_cast<int>(std::round(ref_y));
                const int rw = static_cast<int>(std::round(ref_w));
                const int rh = static_cast<int>(std::round(ref_h));
                if (x0 < 0 || y0 < 0 || x0 + rw > img_w || y0 + rh > img_h || rw <= 0 || rh <= 0) {
                    refs_ok = false;
                    break;
                }

                const int inset_x = std::max(1, rw / 4);
                const int inset_y = std::max(1, rh / 4);
                observed_refs[k] = cv::mean(aligned(cv::Rect(
                    x0 + inset_x,
                    y0 + inset_y,
                    std::max(1, rw - 2 * inset_x),
                    std::max(1, rh - 2 * inset_y))));
            }

            if (refs_ok) {
                classifier_colors = observed_refs;
            }
        }

        auto build_payload = [&](double ox, double oy, double cw, double ch, int rows_local) {
            std::vector<int> cells_local;
            cells_local.reserve(rows_local * cfg.cells_per_row);

            for (int row = 0; row < rows_local; ++row) {
                for (int col = 0; col < cfg.cells_per_row; ++col) {
                    const int x = static_cast<int>(std::round(ox + col * cw));
                    const int y = static_cast<int>(std::round(oy + row * ch));
                    const int cell_px_w = static_cast<int>(std::round(cw));
                    const int cell_px_h = static_cast<int>(std::round(ch));
                    if (x < 0 || y < 0 || x + cell_px_w > img_w || y + cell_px_h > img_h) {
                        return std::vector<uint8_t>{};
                    }

                    const int inset_x = std::max(1, cell_px_w / 4);
                    const int inset_y = std::max(1, cell_px_h / 4);
                    const cv::Rect cell_rect(
                        x + inset_x,
                        y + inset_y,
                        std::max(1, cell_px_w - 2 * inset_x),
                        std::max(1, cell_px_h - 2 * inset_y));
                    const cv::Scalar mean = cv::mean(aligned(cell_rect));

                    int best = 0;
                    int bestd = color_distance_sq(mean, classifier_colors[0]);
                    for (int k = 1; k < 4; ++k) {
                        const int d = color_distance_sq(mean, classifier_colors[k]);
                        if (d < bestd) {
                            best = k;
                            bestd = d;
                        }
                    }
                    cells_local.push_back(best);
                }
            }

            std::vector<uint8_t> payload;
            payload.reserve(cells_local.size() / 4);
            for (size_t i = 0; i + 3 < cells_local.size(); i += 4) {
                uint8_t byte = 0;
                for (int j = 0; j < 4; ++j) {
                    byte |= static_cast<uint8_t>((cells_local[i + j] & 0x3) << (6 - 2 * j));
                }
                payload.push_back(byte);
            }
            return payload;
        };

        auto header_distance = [](const std::vector<uint8_t>& payload) {
            constexpr uint8_t target[5] = {0x43, 0x4D, 0x41, 0x43, 0x01};
            if (payload.size() < 5) {
                return 9999;
            }

            int score = 0;
            for (size_t i = 0; i < 5; ++i) {
                uint8_t diff = static_cast<uint8_t>(payload[i] ^ target[i]);
                while (diff != 0) {
                    score += diff & 1u;
                    diff >>= 1;
                }
            }
            return score;
        };

        struct GridVariant {
            double dx;
            double dy;
            double sx;
            double sy;
            int row_delta;
        };

        std::vector<GridVariant> variants = {{0.0, 0.0, 1.0, 1.0, 0}};
        if (std::string(label) == "warped") {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.15, 0.0, 1.0, 1.0, 0}, {0.15, 0.0, 1.0, 1.0, 0},
                {0.0, -0.15, 1.0, 1.0, 0}, {0.0, 0.15, 1.0, 1.0, 0},
                {-0.10, -0.10, 1.0, 1.0, 0}, {0.10, 0.10, 1.0, 1.0, 0},
                {0.0, 0.0, 0.985, 0.985, 0}, {0.0, 0.0, 1.015, 1.015, 0},
                {0.0, 0.0, 1.0, 1.0, -1}, {0.0, 0.0, 1.0, 1.0, 1},
                {0.0, 0.0, 0.985, 0.985, -1}, {0.0, 0.0, 1.015, 1.015, 1},
            };
        } else if (std::string(label) == "cropped") {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.15, 0.0, 1.0, 1.0, 0}, {0.15, 0.0, 1.0, 1.0, 0},
                {0.0, -0.15, 1.0, 1.0, 0}, {0.0, 0.15, 1.0, 1.0, 0},
                {0.0, 0.0, 0.985, 0.985, 0}, {0.0, 0.0, 1.015, 1.015, 0},
                {0.0, 0.0, 1.0, 1.0, -1}, {0.0, 0.0, 1.0, 1.0, 1},
            };
        }

        int best_score = std::numeric_limits<int>::max();
        std::vector<uint8_t> best_payload;
        for (const auto& variant : variants) {
            const int trial_rows = rows + variant.row_delta;
            if (trial_rows <= 0) {
                continue;
            }

            std::vector<uint8_t> payload = build_payload(
                origin_x + variant.dx * cell_w,
                origin_y + variant.dy * cell_h,
                cell_w * variant.sx,
                cell_h * variant.sy,
                trial_rows);
            if (payload.empty()) {
                continue;
            }

            const int score = header_distance(payload);
            if (score < best_score) {
                best_score = score;
                best_payload = std::move(payload);
                if (score == 0) {
                    break;
                }
            }
        }

        if (best_payload.empty()) {
            return false;
        }

        out_payload = std::move(best_payload);
        return true;
    };

    if (!exact_raw_layout) {
        cv::Mat warped_first;
        if (warp_with_finders(frame, warped_first) && sample_aligned(warped_first, "warped")) {
            return true;
        }

        cv::Mat cropped_first = center_crop_square(frame);
        if (sample_aligned(cropped_first, "cropped")) {
            return true;
        }
    }

    if (exact_raw_layout && sample_aligned(frame, "raw")) {
        return true;
    }

    cv::Mat warped;
    if (warp_with_finders(frame, warped) && sample_aligned(warped, "warped")) {
        return true;
    }

    return sample_aligned(center_crop_square(frame), "cropped");
}

double laplacian_variance(const cv::Mat& img) {
    cv::Mat gray;
    if (img.channels() == 3) {
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = img;
    }

    cv::Mat lap;
    cv::Laplacian(gray, lap, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev[0] * stddev[0];
}

cv::Vec3d compute_color_scale(
    const std::array<cv::Scalar, 4>& expected,
    const std::array<cv::Scalar, 4>& observed) {
    double sb = 0.0;
    double sg = 0.0;
    double sr = 0.0;
    int count = 0;

    for (int i = 0; i < 4; ++i) {
        if (expected[i][0] > 1e-6) sb += observed[i][0] / expected[i][0];
        if (expected[i][1] > 1e-6) sg += observed[i][1] / expected[i][1];
        if (expected[i][2] > 1e-6) sr += observed[i][2] / expected[i][2];
        ++count;
    }

    if (count == 0) {
        return cv::Vec3d(1.0, 1.0, 1.0);
    }
    return cv::Vec3d(sb / count, sg / count, sr / count);
}

} // namespace camcom
