//实现视觉帧渲染、透视矫正与网格采样解码逻辑。

#include "codec.hpp"
#include "common.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace camcom {

    static inline int color_distance_sq(const cv::Scalar& a, const cv::Scalar& b) {
        int db = static_cast<int>(a[0] - b[0]);
        int dg = static_cast<int>(a[1] - b[1]);
        int dr = static_cast<int>(a[2] - b[2]);
        return db * db + dg * dg + dr * dr;
    }

    void render_frame(cv::Mat& out, const std::vector<uint8_t>& payload, const EncoderConfig& cfg) {
        // 将载荷字节转换为 2-bit 单元（每字节内按高位优先）
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

        const int marker_cells = FINDER_MARKER_CELLS; // 定位标记边长（按单元格计）
        const int marker_px = marker_cells * cfg.cell_size;
        const int data_w = cells_per_row * cfg.cell_size;
        const int data_h = rows * cfg.cell_size;

        const int img_w = data_w + marker_px * 2;
        const int img_h = data_h + marker_px * 2;

        out.create(img_h, img_w, CV_8UC3);
        // 背景填充为灰色
        out.setTo(cv::Scalar(255, 255, 255));

        // 在四角绘制定位标记（同心方块）
        auto draw_finder = [&](int x, int y) {
            cv::Rect r(x, y, marker_px, marker_px);
            // 外层黑色
            cv::rectangle(out, r, cv::Scalar(0, 0, 0), cv::FILLED);
            // 内层白色
            int inner = cfg.cell_size;
            cv::rectangle(out, cv::Rect(x + inner, y + inner, marker_px - 2 * inner, marker_px - 2 * inner), cv::Scalar(255, 255, 255), cv::FILLED);
            // 中心黑色
            int inner2 = inner * 2;
            cv::rectangle(out, cv::Rect(x + inner2, y + inner2, marker_px - 2 * inner2, marker_px - 2 * inner2), cv::Scalar(0, 0, 0), cv::FILLED);
            };

        draw_finder(0, 0);
        draw_finder(img_w - marker_px, 0);
        draw_finder(0, img_h - marker_px);
        draw_finder(img_w - marker_px, img_h - marker_px);

        // 在四条边中心增加辅助定位点，提升手机拍摄场景的鲁棒性。
        const int small_marker_cells = 2;
        const int small_marker_px = small_marker_cells * cfg.cell_size;
        auto draw_small = [&](int x, int y) {
            cv::Rect r(x, y, small_marker_px, small_marker_px);
            cv::rectangle(out, r, cv::Scalar(0, 0, 0), cv::FILLED);
            const int inner = std::max(1, cfg.cell_size / 2);
            cv::rectangle(out, cv::Rect(x + inner, y + inner, small_marker_px - 2 * inner, small_marker_px - 2 * inner), cv::Scalar(255, 255, 255), cv::FILLED);
        };
        draw_small((img_w - small_marker_px) / 2, 0);
        draw_small((img_w - small_marker_px) / 2, img_h - small_marker_px);
        draw_small(0, (img_h - small_marker_px) / 2);
        draw_small(img_w - small_marker_px, (img_h - small_marker_px) / 2);

        // 绘制数据单元格：填充定位边距内的数据区域
        const int origin_x = marker_px;
        const int origin_y = marker_px;

        for (int idx = 0; idx < total_cells; ++idx) {
            int r = idx / cells_per_row;
            int c = idx % cells_per_row;
            int x = origin_x + c * cfg.cell_size;
            int y = origin_y + r * cfg.cell_size;
            cv::Rect cell_rect(x, y, cfg.cell_size, cfg.cell_size);
            int v = cells[idx];
            cv::rectangle(out, cell_rect, cfg.colors[v], cv::FILLED);
        }

        // 绘制参考色块（顶部边距内，靠近左上）
        const int ref_cells = cfg.reference_block_size;
        const int ref_px = ref_cells * cfg.cell_size;
        int ref_origin_x = origin_x;
        int ref_origin_y = origin_y - ref_px - cfg.cell_size; // 位于数据区上方
        if (ref_origin_y < cfg.cell_size) ref_origin_y = origin_y; // 回退处理
        for (int k = 0; k < 4; ++k) {
            int rx = ref_origin_x + k * (ref_px + cfg.cell_size / 2);
            cv::Rect r(rx, ref_origin_y, ref_px, ref_px);
            cv::rectangle(out, r, cfg.colors[k], cv::FILLED);
        }
    }

    static bool warp_with_finders(const cv::Mat& src, cv::Mat& warped) {
        cv::Mat gray;
        if (src.channels() == 3) cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
        else gray = src;
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
        cv::Mat bin;
        cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        auto warp_from_quad = [&](const std::array<cv::Point2f, 4>& quad) -> bool {
            auto order_quad = [](const std::array<cv::Point2f, 4>& in,
                                 cv::Point2f& tl,
                                 cv::Point2f& tr,
                                 cv::Point2f& bl,
                                 cv::Point2f& br) {
                std::array<cv::Point2f, 4> p = in;
                std::sort(p.begin(), p.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                    if (a.y != b.y) return a.y < b.y;
                    return a.x < b.x;
                });

                std::array<cv::Point2f, 2> top = { p[0], p[1] };
                std::array<cv::Point2f, 2> bottom = { p[2], p[3] };
                if (top[0].x > top[1].x) std::swap(top[0], top[1]);
                if (bottom[0].x > bottom[1].x) std::swap(bottom[0], bottom[1]);

                tl = top[0]; tr = top[1];
                bl = bottom[0]; br = bottom[1];
            };

            cv::Point2f tl, tr, bl, br;
            order_quad(quad, tl, tr, bl, br);
            const double w1 = cv::norm(tr - tl);
            const double w2 = cv::norm(br - bl);
            const double h1 = cv::norm(bl - tl);
            const double h2 = cv::norm(br - tr);
            const double target_w = std::max(w1, w2);
            const double target_h = std::max(h1, h2);
            if (target_w < 1.0 || target_h < 1.0) return false;

            const int dst_w = static_cast<int>(std::ceil(target_w));
            const int dst_h = static_cast<int>(std::ceil(target_h));
            std::vector<cv::Point2f> dst = {
                cv::Point2f(0.f, 0.f),
                cv::Point2f(static_cast<float>(dst_w - 1), 0.f),
                cv::Point2f(0.f, static_cast<float>(dst_h - 1)),
                cv::Point2f(static_cast<float>(dst_w - 1), static_cast<float>(dst_h - 1))
            };
            cv::Mat M = cv::getPerspectiveTransform(std::vector<cv::Point2f>{ tl, tr, bl, br }, dst);
            cv::warpPerspective(src, warped, M, cv::Size(dst_w, dst_h));
            return true;
        };

        auto warp_from_big_quad = [&]() -> bool {
            cv::Mat edges;
            cv::Canny(gray, edges, 80, 200);

            std::vector<std::vector<cv::Point>> ecs;
            cv::findContours(edges, ecs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double best_area = 0.0;
            std::array<cv::Point2f, 4> best_quad{};
            for (const auto& c : ecs) {
                const double area = std::abs(cv::contourArea(c));
                if (area < static_cast<double>(src.cols) * static_cast<double>(src.rows) * 0.03) continue;

                std::vector<cv::Point> poly;
                cv::approxPolyDP(c, poly, 0.02 * cv::arcLength(c, true), true);
                if (poly.size() != 4 || !cv::isContourConvex(poly)) continue;

                cv::Rect b = cv::boundingRect(poly);
                const double asp = static_cast<double>(b.width) / std::max(1, b.height);
                if (asp < 0.5 || asp > 2.0) continue;

                if (area > best_area) {
                    best_area = area;
                    for (int i = 0; i < 4; ++i) {
                        best_quad[i] = cv::Point2f(static_cast<float>(poly[i].x), static_cast<float>(poly[i].y));
                    }
                }
            }

            if (best_area <= 0.0) return false;
            return warp_from_quad(best_quad);
        };

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        struct Candidate { double area; std::vector<cv::Point> poly; cv::Point2f center; };
        std::vector<Candidate> candidates;

        auto collect_candidates = [&](bool require_nested) {
            std::vector<Candidate> out;
            for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
                const auto& c = contours[i];
                if (require_nested) {
                    if (i >= static_cast<int>(hierarchy.size()) || hierarchy[i][2] < 0) continue;
                    const int child = hierarchy[i][2];
                    if (child < 0 || child >= static_cast<int>(hierarchy.size()) || hierarchy[child][2] < 0) continue;
                }

                double area = cv::contourArea(c);
                if (area < 100.0) continue;

                std::vector<cv::Point> poly;
                cv::approxPolyDP(c, poly, 0.05 * cv::arcLength(c, true), true);
                if (poly.size() < 4) continue;

                cv::Rect bounds = cv::boundingRect(poly);
                double aspect = static_cast<double>(bounds.width) / std::max(1, bounds.height);
                if (aspect < 0.5 || aspect > 2.0) continue;

                double fill_ratio = area / std::max(1.0, static_cast<double>(bounds.area()));
                if (fill_ratio < 0.45 || fill_ratio > 1.05) continue;

                cv::Moments m = cv::moments(poly);
                if (std::abs(m.m00) < 1e-6) continue;
                cv::Point2f center(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                out.push_back({ area, poly, center });
            }
            return out;
        };

        candidates = collect_candidates(true);
        if (candidates.size() < 4) {
            // 回退：有些帧层级结构被压缩噪声破坏，放宽到形状过滤。
            candidates = collect_candidates(false);
        }

        if (candidates.size() < 4) return warp_from_big_quad();
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) { return a.area > b.area; });
        if (candidates.size() > 24) candidates.resize(24);

        // 基于候选面积中位数做区间比例筛选，避免固定阈值在拍摄尺度变化时不稳定。
        const int stat_n = std::min(8, static_cast<int>(candidates.size()));
        std::vector<double> area_stats;
        area_stats.reserve(stat_n);
        for (int i = 0; i < stat_n; ++i) area_stats.push_back(candidates[i].area);
        std::sort(area_stats.begin(), area_stats.end());
        const double median_area = area_stats[stat_n / 2];
        const double min_area = median_area * 0.35;
        const double max_area = median_area * 2.8;
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](const Candidate& c) {
            return c.area < min_area || c.area > max_area;
            }), candidates.end());

        if (candidates.size() < 4) return warp_from_big_quad();

        auto angle_deg = [](const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
            cv::Point2f v1 = a - b;
            cv::Point2f v2 = c - b;
            const double n1 = std::max(1e-6, static_cast<double>(cv::norm(v1)));
            const double n2 = std::max(1e-6, static_cast<double>(cv::norm(v2)));
            const double dot = static_cast<double>(v1.x) * v2.x + static_cast<double>(v1.y) * v2.y;
            double cs = dot / (n1 * n2);
            cs = std::max(-1.0, std::min(1.0, cs));
            return std::acos(cs) * 180.0 / 3.14159265358979323846;
        };

        auto order_quad = [](const std::array<cv::Point2f, 4>& in,
                             cv::Point2f& tl,
                             cv::Point2f& tr,
                             cv::Point2f& bl,
                             cv::Point2f& br) {
            std::array<cv::Point2f, 4> p = in;
            std::sort(p.begin(), p.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
                if (a.y != b.y) return a.y < b.y;
                return a.x < b.x;
            });

            std::array<cv::Point2f, 2> top = { p[0], p[1] };
            std::array<cv::Point2f, 2> bottom = { p[2], p[3] };
            if (top[0].x > top[1].x) std::swap(top[0], top[1]);
            if (bottom[0].x > bottom[1].x) std::swap(bottom[0], bottom[1]);

            tl = top[0]; tr = top[1];
            bl = bottom[0]; br = bottom[1];
        };

        double best_score = -1.0;
        cv::Point2f tl, tr, bl, br;
        const int combo_n = std::min(12, static_cast<int>(candidates.size()));
        for (int i = 0; i < combo_n; ++i) {
            for (int j = i + 1; j < combo_n; ++j) {
                for (int k = j + 1; k < combo_n; ++k) {
                    for (int l = k + 1; l < combo_n; ++l) {
                        std::array<cv::Point2f, 4> pts = {
                            candidates[i].center,
                            candidates[j].center,
                            candidates[k].center,
                            candidates[l].center
                        };

                        cv::Point2f ctl, ctr, cbl, cbr;
                        order_quad(pts, ctl, ctr, cbl, cbr);

                        std::vector<cv::Point2f> poly = { ctl, ctr, cbr, cbl };
                        const double area = std::abs(cv::contourArea(poly));
                        if (area < 1000.0) continue;

                        const double w1 = cv::norm(ctr - ctl);
                        const double w2 = cv::norm(cbr - cbl);
                        const double h1 = cv::norm(cbl - ctl);
                        const double h2 = cv::norm(cbr - ctr);
                        if (std::min({ w1, w2, h1, h2 }) < 1.0) continue;

                        const double wr = std::max(w1, w2) / std::max(1e-6, std::min(w1, w2));
                        const double hr = std::max(h1, h2) / std::max(1e-6, std::min(h1, h2));
                        if (wr > 1.9 || hr > 1.9) continue;

                        const double a0 = angle_deg(cbl, ctl, ctr);
                        const double a1 = angle_deg(ctl, ctr, cbr);
                        const double a2 = angle_deg(ctr, cbr, cbl);
                        const double a3 = angle_deg(cbr, cbl, ctl);
                        const double ang_dev =
                            std::abs(a0 - 90.0) + std::abs(a1 - 90.0) +
                            std::abs(a2 - 90.0) + std::abs(a3 - 90.0);
                        if (ang_dev > 120.0) continue;

                        const double cand_max = std::max({ candidates[i].area, candidates[j].area, candidates[k].area, candidates[l].area });
                        const double cand_min = std::max(1.0, std::min({ candidates[i].area, candidates[j].area, candidates[k].area, candidates[l].area }));
                        const double area_ratio = cand_max / cand_min;
                        if (area_ratio > 4.0) continue;

                        const double score = area / (1.0 + 0.02 * ang_dev + 0.8 * (wr - 1.0) + 0.8 * (hr - 1.0) + 0.3 * (area_ratio - 1.0));
                        if (score > best_score) {
                            best_score = score;
                            tl = ctl; tr = ctr; bl = cbl; br = cbr;
                        }
                    }
                }
            }
        }

        if (best_score <= 0.0) return warp_from_big_quad();

        return warp_from_quad(std::array<cv::Point2f, 4>{ tl, tr, bl, br });
    }

    static cv::Mat center_crop_square(const cv::Mat& src) {
        int side = std::min(src.cols, src.rows);
        int x = (src.cols - side) / 2;
        int y = (src.rows - side) / 2;
        cv::Rect roi(x, y, side, side);
        return src(roi).clone();
    }

    bool sample_frame(const cv::Mat& frame, std::vector<uint8_t>& out_payload, const EncoderConfig& cfg) {
        auto sample_aligned = [&](const cv::Mat& aligned, const char* label) -> bool {
            const int img_w = aligned.cols;
            const int img_h = aligned.rows;

            int origin_x = 0, origin_y = 0;
            double cell_w = cfg.cell_size;
            double cell_h = cfg.cell_size;
            int rows = 0;

            if (std::string(label) == "warped") {
                // 渲染尺寸是 data + 两侧 finder 边距，warp 后应按同一几何关系采样。
                cell_w = static_cast<double>(img_w) / (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS);
                cell_h = cell_w; // 假设单元格为正方形
                origin_x = static_cast<int>(std::round(FINDER_MARKER_CELLS * cell_w));
                origin_y = static_cast<int>(std::round(FINDER_MARKER_CELLS * cell_h));
                rows = static_cast<int>(std::round(img_h / cell_h)) - 2 * FINDER_MARKER_CELLS;
            } else if (std::string(label) == "raw" && img_w == (cfg.cells_per_row + 2 * FINDER_MARKER_CELLS) * cfg.cell_size) {
                origin_x = FINDER_MARKER_CELLS * cfg.cell_size;
                origin_y = FINDER_MARKER_CELLS * cfg.cell_size;
                rows = (img_h - 2 * FINDER_MARKER_CELLS * cfg.cell_size) / cfg.cell_size;
            } else {
                cv::Mat gray;
                if (aligned.channels() == 3) cv::cvtColor(aligned, gray, cv::COLOR_BGR2GRAY); else gray = aligned;

                std::vector<double> row_std(img_h), col_std(img_w);
                for (int y = 0; y < img_h; ++y) {
                    cv::Scalar mean, stddev;
                    cv::meanStdDev(gray.row(y), mean, stddev);
                    row_std[y] = stddev[0];
                }
                for (int x = 0; x < img_w; ++x) {
                    cv::Scalar mean, stddev;
                    cv::meanStdDev(gray.col(x), mean, stddev);
                    col_std[x] = stddev[0];
                }

                auto find_band = [](const std::vector<double>& v, double thresh_frac) {
                    double maxv = 0; for (double d : v) maxv = std::max(maxv, d);
                    double th = maxv * thresh_frac;
                    int lo = -1, hi = -1;
                    for (int i = 0; i < static_cast<int>(v.size()); ++i) {
                        if (v[i] >= th) { if (lo == -1) lo = i; hi = i; }
                    }
                    return std::pair<int,int>(lo, hi);
                };

                auto [row_lo, row_hi] = find_band(row_std, 0.3);
                auto [col_lo, col_hi] = find_band(col_std, 0.3);

                if (row_lo == -1 || col_lo == -1) {
                    std::cout << "[decoder] sample_frame(" << label << ") 未找到有效方差带\n";
                    return false;
                }

                origin_x = std::max(0, col_lo - (col_lo % cfg.cell_size));
                origin_y = std::max(0, row_lo - (row_lo % cfg.cell_size));
                int data_w = cfg.cells_per_row * cfg.cell_size;
                if (origin_x + data_w > img_w) origin_x = std::max(0, img_w - data_w);

                const double avail_w = static_cast<double>(img_w - origin_x);
                const double max_cell_w = avail_w / std::max(1, cfg.cells_per_row);
                if (max_cell_w > 0.0 && max_cell_w < cell_w) {
                    cell_w = max_cell_w;
                    cell_h = cell_w;
                }

                int observed_h = row_hi - origin_y + 1;
                rows = static_cast<int>(std::floor(observed_h / std::max(1e-6, cell_h)));
            }

            if (rows <= 0) {
                    std::cout << "[decoder] sample_frame(" << label << ") 行数 <= 0\n";
                return false;
            }

            const int cols = cfg.cells_per_row;
            std::vector<int> cells;
            cells.reserve(rows * cols);

            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {
                    int x = origin_x + static_cast<int>(std::round(c * cell_w));
                    int y = origin_y + static_cast<int>(std::round(r * cell_h));
                    int cell_px_w = static_cast<int>(std::round(cell_w));
                    int cell_px_h = static_cast<int>(std::round(cell_h));

                    if (x + cell_px_w > img_w || y + cell_px_h > img_h) {
                        std::cout << "[decoder] sample_frame(" << label << ") 单元格越界\n";
                        return false;
                    }

                    cv::Rect cell_rect(x, y, cell_px_w, cell_px_h);
                    cv::Mat roi = aligned(cell_rect);
                    cv::Scalar mean = cv::mean(roi);
                    int best = 0;
                    int bestd = color_distance_sq(mean, cfg.colors[0]);
                    for (int k = 1; k < 4; ++k) {
                        int d = color_distance_sq(mean, cfg.colors[k]);
                        if (d < bestd) { best = k; bestd = d; }
                    }
                    cells.push_back(best);
                }
            }

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
        };

        cv::Mat warped;
        if (warp_with_finders(frame, warped)) {
            if (sample_aligned(warped, "warped")) {
                std::cout << "[decoder] sample_frame path=warped\n";
                return true;
            }
            std::cout << "[decoder] sample_frame: 透视矫正采样失败，尝试 raw\n";
        } else {
            std::cout << "[decoder] warp_with_finders 失败，尝试 raw\n";
        }

        // 回退方案：先中心裁剪为正方形后再采样
        cv::Mat cropped = center_crop_square(frame);
        if (sample_aligned(cropped, "cropped")) {
            std::cout << "[decoder] sample_frame path=cropped\n";
            return true;
        }

        // 最后回退到 raw 路径（测试场景通常为严格对齐图像）
        if (sample_aligned(frame, "raw")) {
            std::cout << "[decoder] sample_frame path=raw\n";
            return true;
        }

        return false;
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
        // 计算每个通道的缩放系数：按颜色的观测值/期望值取平均，并避免除零
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
