// 实现可视化帧渲染、定位点检测、透视矫正与网格采样逻辑。
#include "codec.hpp"
#include "common.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace camcom {
namespace {

bool find_corner_markers(const cv::Mat& aligned, std::array<cv::Rect, 4>& markers);

bool has_raw_layout_corners(const cv::Mat& frame, const EncoderConfig& cfg) {
    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) {
        return false;
    }

    const int patch_w = std::max(1, cfg.cell_size);
    const int patch_h = std::max(1, cfg.cell_size);
    const std::array<cv::Rect, 4> corner_patches = {
        cv::Rect(0, 0, std::min(patch_w, frame.cols), std::min(patch_h, frame.rows)),
        cv::Rect(std::max(0, frame.cols - patch_w), 0, std::min(patch_w, frame.cols), std::min(patch_h, frame.rows)),
        cv::Rect(0, std::max(0, frame.rows - patch_h), std::min(patch_w, frame.cols), std::min(patch_h, frame.rows)),
        cv::Rect(
            std::max(0, frame.cols - patch_w),
            std::max(0, frame.rows - patch_h),
            std::min(patch_w, frame.cols),
            std::min(patch_h, frame.rows)),
    };

    for (const auto& patch : corner_patches) {
        if (patch.width <= 0 || patch.height <= 0) {
            return false;
        }
        const cv::Scalar mean = cv::mean(frame(patch));
        const double intensity = (mean[0] + mean[1] + mean[2]) / 3.0;
        if (intensity > 64.0) {
            return false;
        }
    }
    return true;
}

struct FinderCandidate {
    double area = 0.0;
    double score = 0.0;
    cv::Point2f center;
    cv::RotatedRect rect;
    std::array<cv::Point2f, 4> box{};
};

struct LayoutCell {
    int row = 0;
    int col = 0;
};

int layout_total_cols(const EncoderConfig& cfg) {
    return cfg.cells_per_row + 2 * FINDER_MARKER_CELLS;
}

int layout_ref_top_row() {
    return 1;
}

int layout_ref_left_col() {
    return FINDER_MARKER_CELLS + 2;
}

bool cell_in_rect(int row, int col, int top, int left, int height, int width) {
    return row >= top && row < top + height && col >= left && col < left + width;
}

void reserve_rect(
    std::vector<uint8_t>& mask,
    int total_rows,
    int total_cols,
    int top,
    int left,
    int height,
    int width,
    int padding = 0) {
    const int r0 = std::max(0, top - padding);
    const int c0 = std::max(0, left - padding);
    const int r1 = std::min(total_rows, top + height + padding);
    const int c1 = std::min(total_cols, left + width + padding);
    for (int row = r0; row < r1; ++row) {
        for (int col = c0; col < c1; ++col) {
            mask[static_cast<size_t>(row) * static_cast<size_t>(total_cols) + static_cast<size_t>(col)] = 1;
        }
    }
}

std::vector<uint8_t> build_reserved_cell_mask(int total_rows, const EncoderConfig& cfg) {
    const int total_cols = layout_total_cols(cfg);
    std::vector<uint8_t> mask(static_cast<size_t>(total_rows) * static_cast<size_t>(total_cols), 0);

    const int marker = FINDER_MARKER_CELLS;
    const int small = 2;
    const int ref = std::max(1, cfg.reference_block_size);

    reserve_rect(mask, total_rows, total_cols, 0, 0, marker, marker, 1);
    reserve_rect(mask, total_rows, total_cols, 0, total_cols - marker, marker, marker, 1);
    reserve_rect(mask, total_rows, total_cols, total_rows - marker, 0, marker, marker, 1);
    reserve_rect(mask, total_rows, total_cols, total_rows - marker, total_cols - marker, marker, marker, 1);

    const int top_small_col = std::max(0, (total_cols - small) / 2);
    const int left_small_row = std::max(0, (total_rows - small) / 2);
    reserve_rect(mask, total_rows, total_cols, 0, top_small_col, small, small, 1);
    reserve_rect(mask, total_rows, total_cols, total_rows - small, top_small_col, small, small, 1);
    reserve_rect(mask, total_rows, total_cols, left_small_row, 0, small, small, 1);
    reserve_rect(mask, total_rows, total_cols, left_small_row, total_cols - small, small, small, 1);

    const int ref_row = layout_ref_top_row();
    const int ref_col0 = layout_ref_left_col();
    const int ref_stride = ref + 1;
    for (int k = 0; k < 4; ++k) {
        reserve_rect(mask, total_rows, total_cols, ref_row, ref_col0 + k * ref_stride, ref, ref, 1);
    }

    return mask;
}

std::vector<LayoutCell> build_data_cells(int total_rows, const EncoderConfig& cfg) {
    const int total_cols = layout_total_cols(cfg);
    const std::vector<uint8_t> reserved = build_reserved_cell_mask(total_rows, cfg);
    std::vector<LayoutCell> cells;
    cells.reserve(static_cast<size_t>(total_rows) * static_cast<size_t>(total_cols));
    for (int row = 0; row < total_rows; ++row) {
        for (int col = 0; col < total_cols; ++col) {
            if (reserved[static_cast<size_t>(row) * static_cast<size_t>(total_cols) + static_cast<size_t>(col)] != 0) {
                continue;
            }
            cells.push_back(LayoutCell{row, col});
        }
    }
    return cells;
}

int layout_capacity_cells(int total_rows, const EncoderConfig& cfg) {
    return static_cast<int>(build_data_cells(total_rows, cfg).size());
}

int minimum_total_rows_for_cells(int payload_cells, const EncoderConfig& cfg) {
    const int min_rows = 2 * FINDER_MARKER_CELLS + 2;
    int total_rows = min_rows;
    while (layout_capacity_cells(total_rows, cfg) < payload_cells) {
        ++total_rows;
    }
    return total_rows;
}

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

std::array<cv::Point2f, 4> order_quad_points(const std::vector<cv::Point2f>& points) {
    std::array<cv::Point2f, 4> ordered{};
    std::vector<cv::Point2f> pts = points;
    std::sort(pts.begin(), pts.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
    });

    ordered[0] = pts[0].x < pts[1].x ? pts[0] : pts[1];
    ordered[1] = pts[0].x < pts[1].x ? pts[1] : pts[0];
    ordered[2] = pts[2].x < pts[3].x ? pts[2] : pts[3];
    ordered[3] = pts[2].x < pts[3].x ? pts[3] : pts[2];
    return ordered;
}

std::array<cv::Point2f, 4> order_rect_points(const cv::RotatedRect& rect) {
    cv::Point2f raw[4];
    rect.points(raw);
    return order_quad_points({raw[0], raw[1], raw[2], raw[3]});
}

bool is_square_like(const cv::Size2f& size, double min_aspect = 0.65, double max_aspect = 1.35) {
    const double w = std::max(1.0f, size.width);
    const double h = std::max(1.0f, size.height);
    const double aspect = std::max(w, h) / std::min(w, h);
    return aspect >= min_aspect && aspect <= max_aspect;
}

void expected_canvas_size(const EncoderConfig& cfg, int& target_w, int& target_h) {
    const int frame_header_bytes = 4 + 1 + 4 + 4 + 4 + 4;
    const int max_codeword_bytes = frame_header_bytes + cfg.payload_bytes_per_frame;
    const int max_cells = max_codeword_bytes * 4;
    const int total_rows = minimum_total_rows_for_cells(max_cells, cfg);
    target_w = layout_total_cols(cfg) * cfg.cell_size;
    target_h = total_rows * cfg.cell_size;
}

cv::Rect clamp_rect_to_image(const cv::Rect& rect, const cv::Size& image_size) {
    const cv::Rect bounds(0, 0, image_size.width, image_size.height);
    return rect & bounds;
}

bool detect_finder_candidates(const cv::Mat& src, std::vector<FinderCandidate>& out) {
    if (src.empty()) {
        return false;
    }

    cv::Mat gray;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    const double min_area = std::max(100.0, src.total() * 0.00025);
    std::vector<FinderCandidate> candidates;
    candidates.reserve(24);

    auto append_outer_dark_candidates = [&](const cv::Mat& mask) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty() || hierarchy.empty()) {
            return;
        }

        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            const int child = hierarchy[i][2];
            if (child < 0) {
                continue;
            }
            const int grand_child = hierarchy[child][2];
            if (grand_child < 0) {
                continue;
            }

            const double outer_area = std::abs(cv::contourArea(contours[i]));
            const double child_area = std::abs(cv::contourArea(contours[child]));
            const double grand_area = std::abs(cv::contourArea(contours[grand_child]));
            if (outer_area < min_area || child_area <= 0.0 || grand_area <= 0.0) {
                continue;
            }

            const cv::RotatedRect outer_rect = cv::minAreaRect(contours[i]);
            const cv::RotatedRect child_rect = cv::minAreaRect(contours[child]);
            const cv::RotatedRect grand_rect = cv::minAreaRect(contours[grand_child]);
            if (!is_square_like(outer_rect.size) ||
                !is_square_like(child_rect.size, 0.55, 1.45) ||
                !is_square_like(grand_rect.size, 0.55, 1.45)) {
                continue;
            }

            const double outer_side =
                (std::max(outer_rect.size.width, 1.0f) + std::max(outer_rect.size.height, 1.0f)) * 0.5;
            const double child_side =
                (std::max(child_rect.size.width, 1.0f) + std::max(child_rect.size.height, 1.0f)) * 0.5;
            const double grand_side =
                (std::max(grand_rect.size.width, 1.0f) + std::max(grand_rect.size.height, 1.0f)) * 0.5;

            const double outer_to_child = outer_side / std::max(1.0, child_side);
            const double child_to_grand = child_side / std::max(1.0, grand_side);
            const double center_offset =
                cv::norm(outer_rect.center - child_rect.center) / std::max(1.0, outer_side) +
                cv::norm(outer_rect.center - grand_rect.center) / std::max(1.0, outer_side);
            const double child_fill = child_area / std::max(1.0, outer_area);
            const double grand_fill = grand_area / std::max(1.0, outer_area);
            const double rect_fill = outer_area /
                std::max(1.0, static_cast<double>(outer_rect.size.width) * outer_rect.size.height);

            const bool sane =
                outer_to_child >= 1.20 && outer_to_child <= 1.70 &&
                child_to_grand >= 1.35 && child_to_grand <= 2.20 &&
                center_offset <= 0.35 &&
                child_fill >= 0.22 && child_fill <= 0.70 &&
                grand_fill >= 0.04 && grand_fill <= 0.38 &&
                rect_fill >= 0.35 && rect_fill <= 1.15;
            if (!sane) {
                continue;
            }

            const double ratio_penalty =
                std::abs(outer_to_child - (7.0 / 5.0)) +
                std::abs(child_to_grand - (5.0 / 3.0));
            FinderCandidate candidate;
            candidate.area = outer_area;
            candidate.center = outer_rect.center;
            candidate.rect = outer_rect;
            candidate.box = order_rect_points(outer_rect);
            candidate.score = outer_area - 1800.0 * ratio_penalty - 2200.0 * center_offset;
            candidates.push_back(candidate);
        }
    };

    auto append_inner_white_candidates = [&](const cv::Mat& mask) {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty() || hierarchy.empty()) {
            return;
        }

        for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
            const int child = hierarchy[i][2];
            if (child < 0) {
                continue;
            }

            const double inner_area = std::abs(cv::contourArea(contours[i]));
            const double center_area = std::abs(cv::contourArea(contours[child]));
            if (inner_area < min_area * 0.30 || center_area <= 0.0) {
                continue;
            }

            const cv::RotatedRect inner_rect = cv::minAreaRect(contours[i]);
            const cv::RotatedRect center_rect = cv::minAreaRect(contours[child]);
            if (!is_square_like(inner_rect.size, 0.60, 1.40) ||
                !is_square_like(center_rect.size, 0.60, 1.40)) {
                continue;
            }

            const double inner_side =
                (std::max(inner_rect.size.width, 1.0f) + std::max(inner_rect.size.height, 1.0f)) * 0.5;
            const double center_side =
                (std::max(center_rect.size.width, 1.0f) + std::max(center_rect.size.height, 1.0f)) * 0.5;
            const double inner_to_center = inner_side / std::max(1.0, center_side);
            const double center_offset =
                cv::norm(inner_rect.center - center_rect.center) / std::max(1.0, inner_side);
            const double center_fill = center_area / std::max(1.0, inner_area);

            const bool sane =
                inner_to_center >= 1.35 && inner_to_center <= 2.20 &&
                center_offset <= 0.20 &&
                center_fill >= 0.15 && center_fill <= 0.55;
            if (!sane) {
                continue;
            }

            const float expand = 7.0f / 5.0f;
            cv::RotatedRect full_rect(
                inner_rect.center,
                cv::Size2f(inner_rect.size.width * expand, inner_rect.size.height * expand),
                inner_rect.angle);
            FinderCandidate candidate;
            candidate.area = inner_area * expand * expand;
            candidate.center = full_rect.center;
            candidate.rect = full_rect;
            candidate.box = order_rect_points(full_rect);
            candidate.score = candidate.area -
                1800.0 * std::abs(inner_to_center - (5.0 / 3.0)) -
                2200.0 * center_offset;
            candidates.push_back(candidate);
        }
    };

    cv::Mat dark_mask;
    cv::threshold(gray, dark_mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    append_outer_dark_candidates(dark_mask);

    cv::Mat bright_mask;
    cv::threshold(gray, bright_mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    append_inner_white_candidates(bright_mask);

    if (candidates.empty()) {
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const FinderCandidate& lhs, const FinderCandidate& rhs) {
        return lhs.score > rhs.score;
    });

    out.clear();
    out.reserve(std::min<size_t>(candidates.size(), 16));
    for (const auto& candidate : candidates) {
        const double candidate_side = std::max(candidate.rect.size.width, candidate.rect.size.height);
        bool duplicate = false;
        for (const auto& kept : out) {
            const double kept_side = std::max(kept.rect.size.width, kept.rect.size.height);
            const double distance = cv::norm(candidate.center - kept.center);
            if (distance < std::max(candidate_side, kept_side) * 0.40) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            out.push_back(candidate);
        }
        if (out.size() >= 16) {
            break;
        }
    }
    return out.size() >= 4;
}

bool select_finder_quad(
    const std::vector<FinderCandidate>& candidates,
    std::array<FinderCandidate, 4>& ordered_quad) {
    if (candidates.size() < 4) {
        return false;
    }

    std::vector<FinderCandidate> pool = candidates;
    if (pool.size() > 12) {
        pool.resize(12);
    }

    bool found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    std::array<FinderCandidate, 4> best_quad{};

    for (int a = 0; a < static_cast<int>(pool.size()); ++a) {
        for (int b = a + 1; b < static_cast<int>(pool.size()); ++b) {
            for (int c = b + 1; c < static_cast<int>(pool.size()); ++c) {
                for (int d = c + 1; d < static_cast<int>(pool.size()); ++d) {
                    std::array<FinderCandidate, 4> quad = {
                        pool[a], pool[b], pool[c], pool[d],
                    };
                    std::sort(quad.begin(), quad.end(), [](const FinderCandidate& lhs, const FinderCandidate& rhs) {
                        return lhs.center.y == rhs.center.y ? lhs.center.x < rhs.center.x : lhs.center.y < rhs.center.y;
                    });

                    const FinderCandidate tl = quad[0].center.x < quad[1].center.x ? quad[0] : quad[1];
                    const FinderCandidate tr = quad[0].center.x < quad[1].center.x ? quad[1] : quad[0];
                    const FinderCandidate bl = quad[2].center.x < quad[3].center.x ? quad[2] : quad[3];
                    const FinderCandidate br = quad[2].center.x < quad[3].center.x ? quad[3] : quad[2];

                    if (tl.center.x >= tr.center.x || bl.center.x >= br.center.x ||
                        tl.center.y >= bl.center.y || tr.center.y >= br.center.y) {
                        continue;
                    }

                    const double top_w = cv::norm(tr.center - tl.center);
                    const double bottom_w = cv::norm(br.center - bl.center);
                    const double left_h = cv::norm(bl.center - tl.center);
                    const double right_h = cv::norm(br.center - tr.center);
                    const double min_edge = std::min({top_w, bottom_w, left_h, right_h});
                    if (min_edge < 10.0) {
                        continue;
                    }

                    const double mean_side =
                        (std::max(tl.rect.size.width, tl.rect.size.height) +
                         std::max(tr.rect.size.width, tr.rect.size.height) +
                         std::max(bl.rect.size.width, bl.rect.size.height) +
                         std::max(br.rect.size.width, br.rect.size.height)) * 0.25;
                    const double size_range =
                        std::max({std::max(tl.rect.size.width, tl.rect.size.height),
                                  std::max(tr.rect.size.width, tr.rect.size.height),
                                  std::max(bl.rect.size.width, bl.rect.size.height),
                                  std::max(br.rect.size.width, br.rect.size.height)}) -
                        std::min({std::max(tl.rect.size.width, tl.rect.size.height),
                                  std::max(tr.rect.size.width, tr.rect.size.height),
                                  std::max(bl.rect.size.width, bl.rect.size.height),
                                  std::max(br.rect.size.width, br.rect.size.height)});

                    const double score =
                        quad_score(tl.center, tr.center, bl.center, br.center) +
                        tl.score + tr.score + bl.score + br.score -
                        1800.0 * (size_range / std::max(1.0, mean_side));
                    if (!found || score > best_score) {
                        found = true;
                        best_score = score;
                        best_quad = {tl, tr, bl, br};
                    }
                }
            }
        }
    }

    if (!found) {
        return false;
    }

    ordered_quad = best_quad;
    return true;
}

bool quad_is_corner_aligned(
    const std::array<FinderCandidate, 4>& quad,
    const cv::Size& image_size,
    double margin_frac = 0.30) {
    const double margin_x = image_size.width * margin_frac;
    const double margin_y = image_size.height * margin_frac;

    return
        quad[0].center.x <= margin_x && quad[0].center.y <= margin_y &&
        quad[1].center.x >= image_size.width - margin_x && quad[1].center.y <= margin_y &&
        quad[2].center.x <= margin_x && quad[2].center.y >= image_size.height - margin_y &&
        quad[3].center.x >= image_size.width - margin_x && quad[3].center.y >= image_size.height - margin_y;
}

bool warp_from_quad_points(
    const cv::Mat& src,
    const std::array<cv::Point2f, 4>& quad,
    const EncoderConfig& cfg,
    cv::Mat& warped) {
    int target_w = 0;
    int target_h = 0;
    expected_canvas_size(cfg, target_w, target_h);
    if (target_w <= 0 || target_h <= 0) {
        return false;
    }

    const std::vector<cv::Point2f> src_pts = {
        quad[0],
        quad[1],
        quad[2],
        quad[3],
    };
    const std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(target_w - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(target_h - 1)),
        cv::Point2f(static_cast<float>(target_w - 1), static_cast<float>(target_h - 1)),
    };

    const cv::Mat transform = cv::getPerspectiveTransform(src_pts, dst_pts);
    cv::warpPerspective(src, warped, transform, cv::Size(target_w, target_h));
    return !warped.empty();
}

std::array<cv::Point2f, 4> quad_points_from_finder_quad(const std::array<FinderCandidate, 4>& quad) {
    return {
        quad[0].box[0],
        quad[1].box[1],
        quad[2].box[2],
        quad[3].box[3],
    };
}

bool warp_from_finder_quad(
    const cv::Mat& src,
    const std::array<FinderCandidate, 4>& quad,
    const EncoderConfig& cfg,
    cv::Mat& warped) {
    return warp_from_quad_points(src, quad_points_from_finder_quad(quad), cfg, warped);
}

bool warp_with_structured_finders(const cv::Mat& src, cv::Mat& warped, const EncoderConfig& cfg) {
    std::vector<FinderCandidate> candidates;
    std::array<FinderCandidate, 4> quad{};
    if (!detect_finder_candidates(src, candidates) || !select_finder_quad(candidates, quad)) {
        return false;
    }
    if (!warp_from_finder_quad(src, quad, cfg, warped)) {
        return false;
    }

    std::vector<FinderCandidate> refined_candidates;
    std::array<FinderCandidate, 4> refined_quad{};
    if (detect_finder_candidates(warped, refined_candidates) &&
        select_finder_quad(refined_candidates, refined_quad) &&
        quad_is_corner_aligned(refined_quad, warped.size())) {
        cv::Mat refined;
        if (warp_from_finder_quad(warped, refined_quad, cfg, refined)) {
            warped = std::move(refined);
        }
    }
    return true;
}

bool warp_with_canvas_quad(const cv::Mat& src, cv::Mat& warped, const EncoderConfig& cfg) {
    cv::Mat gray;
    if (src.channels() == 3) {
        cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = src;
    }
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    cv::Mat bin;
    cv::threshold(gray, bin, 72, 255, cv::THRESH_BINARY);
    cv::morphologyEx(
        bin,
        bin,
        cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int target_w = 0;
    int target_h = 0;
    expected_canvas_size(cfg, target_w, target_h);
    const double target_aspect = static_cast<double>(target_w) / static_cast<double>(target_h);

    double best_area = 0.0;
    std::array<cv::Point2f, 4> best_quad{};
    bool found = false;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < src.total() * 0.08) {
            continue;
        }

        std::vector<cv::Point> poly;
        cv::approxPolyDP(contour, poly, 0.02 * cv::arcLength(contour, true), true);
        if (poly.size() != 4) {
            continue;
        }

        std::vector<cv::Point2f> quad;
        quad.reserve(4);
        for (const auto& p : poly) {
            quad.push_back(cv::Point2f(static_cast<float>(p.x), static_cast<float>(p.y)));
        }
        const auto ordered = order_quad_points(quad);
        const double w1 = cv::norm(ordered[1] - ordered[0]);
        const double w2 = cv::norm(ordered[3] - ordered[2]);
        const double h1 = cv::norm(ordered[2] - ordered[0]);
        const double h2 = cv::norm(ordered[3] - ordered[1]);
        const double aspect =
            std::max(w1, w2) / std::max(1.0, std::max(h1, h2));
        if (aspect < target_aspect * 0.75 || aspect > target_aspect * 1.25) {
            continue;
        }

        if (area > best_area) {
            best_area = area;
            best_quad = ordered;
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    const std::vector<cv::Point2f> dst = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(target_w - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(target_h - 1)),
        cv::Point2f(static_cast<float>(target_w - 1), static_cast<float>(target_h - 1)),
    };
    const cv::Mat transform = cv::getPerspectiveTransform(
        std::vector<cv::Point2f>{best_quad[0], best_quad[1], best_quad[2], best_quad[3]},
        dst);
    cv::warpPerspective(src, warped, transform, cv::Size(target_w, target_h));

    std::array<cv::Rect, 4> markers{};
    if (find_corner_markers(warped, markers)) {
        const std::vector<cv::Point2f> refined_src = {
            cv::Point2f(static_cast<float>(markers[0].x), static_cast<float>(markers[0].y)),
            cv::Point2f(static_cast<float>(markers[1].x + markers[1].width - 1), static_cast<float>(markers[1].y)),
            cv::Point2f(static_cast<float>(markers[2].x), static_cast<float>(markers[2].y + markers[2].height - 1)),
            cv::Point2f(
                static_cast<float>(markers[3].x + markers[3].width - 1),
                static_cast<float>(markers[3].y + markers[3].height - 1)),
        };
        const std::vector<cv::Point2f> refined_dst = {
            cv::Point2f(0.0f, 0.0f),
            cv::Point2f(static_cast<float>(target_w - 1), 0.0f),
            cv::Point2f(0.0f, static_cast<float>(target_h - 1)),
            cv::Point2f(static_cast<float>(target_w - 1), static_cast<float>(target_h - 1)),
        };
        cv::Mat refined;
        const cv::Mat refine_transform = cv::getPerspectiveTransform(refined_src, refined_dst);
        cv::warpPerspective(warped, refined, refine_transform, cv::Size(target_w, target_h));
        if (!refined.empty()) {
            warped = std::move(refined);
        }
    }
    return true;
}

bool warp_with_finders(const cv::Mat& src, cv::Mat& warped, const EncoderConfig& cfg) {
    if (warp_with_structured_finders(src, warped, cfg)) {
        return true;
    }

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
    const double target_w = std::max(w1, w2);
    const double target_h = std::max(h1, h2);
    if (target_w < 1.0 || target_h < 1.0) {
        return false;
    }

    const int dst_w = static_cast<int>(std::ceil(target_w));
    const int dst_h = static_cast<int>(std::ceil(target_h));
    std::vector<cv::Point2f> dst = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(dst_w - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(dst_h - 1)),
        cv::Point2f(static_cast<float>(dst_w - 1), static_cast<float>(dst_h - 1)),
    };

    const cv::Mat transform =
        cv::getPerspectiveTransform(std::vector<cv::Point2f>{tl, tr, bl, br}, dst);
    cv::warpPerspective(src, warped, transform, cv::Size(dst_w, dst_h));
    return true;
}

cv::Mat center_crop_expected_layout(const cv::Mat& src, const EncoderConfig& cfg) {
    int target_w = 0;
    int target_h = 0;
    expected_canvas_size(cfg, target_w, target_h);
    const double expected_aspect =
        static_cast<double>(target_w) / std::max(1.0, static_cast<double>(target_h));

    int crop_w = src.cols;
    int crop_h = static_cast<int>(std::round(crop_w / expected_aspect));
    if (crop_h > src.rows) {
        crop_h = src.rows;
        crop_w = static_cast<int>(std::round(crop_h * expected_aspect));
    }

    crop_w = std::clamp(crop_w, 1, src.cols);
    crop_h = std::clamp(crop_h, 1, src.rows);

    const int x = std::max(0, (src.cols - crop_w) / 2);
    const int y = std::max(0, (src.rows - crop_h) / 2);
    return src(cv::Rect(x, y, crop_w, crop_h)).clone();
}

bool find_corner_markers(
    const cv::Mat& aligned,
    std::array<cv::Rect, 4>& markers) {
    if (aligned.empty()) {
        return false;
    }

    std::vector<FinderCandidate> candidates;
    std::array<FinderCandidate, 4> quad{};
    if (detect_finder_candidates(aligned, candidates) &&
        select_finder_quad(candidates, quad) &&
        quad_is_corner_aligned(quad, aligned.size())) {
        for (int i = 0; i < 4; ++i) {
            markers[i] = clamp_rect_to_image(quad[i].rect.boundingRect(), aligned.size());
            if (markers[i].empty()) {
                return false;
            }
        }
        return true;
    }

    cv::Mat gray;
    if (aligned.channels() == 3) {
        cv::cvtColor(aligned, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = aligned;
    }
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    cv::Mat dark_mask;
    cv::threshold(gray, dark_mask, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    const int roi_w = std::max(32, aligned.cols / 4);
    const int roi_h = std::max(32, aligned.rows / 4);
    const int min_side = std::max(8, std::min(aligned.cols, aligned.rows) / 40);

    struct CornerRoi {
        cv::Rect roi;
        cv::Point2f expected_center;
    };

    const std::array<CornerRoi, 4> rois = {{
        {cv::Rect(0, 0, roi_w, roi_h), cv::Point2f(roi_w * 0.22f, roi_h * 0.22f)},
        {cv::Rect(aligned.cols - roi_w, 0, roi_w, roi_h), cv::Point2f(aligned.cols - roi_w * 0.22f, roi_h * 0.22f)},
        {cv::Rect(0, aligned.rows - roi_h, roi_w, roi_h), cv::Point2f(roi_w * 0.22f, aligned.rows - roi_h * 0.22f)},
        {cv::Rect(aligned.cols - roi_w, aligned.rows - roi_h, roi_w, roi_h),
         cv::Point2f(aligned.cols - roi_w * 0.22f, aligned.rows - roi_h * 0.22f)},
    }};

    for (int corner = 0; corner < 4; ++corner) {
        const cv::Rect roi = rois[corner].roi;
        cv::Mat roi_mask = dark_mask(roi);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double best_score = -1.0;
        cv::Rect best_rect;
        for (const auto& contour : contours) {
            const double area = cv::contourArea(contour);
            if (area < static_cast<double>(min_side * min_side)) {
                continue;
            }

            const cv::Rect local = cv::boundingRect(contour);
            if (local.width < min_side || local.height < min_side) {
                continue;
            }

            const double aspect =
                static_cast<double>(local.width) / std::max(1.0, static_cast<double>(local.height));
            if (aspect < 0.60 || aspect > 1.40) {
                continue;
            }

            const double fill_ratio =
                area / std::max(1.0, static_cast<double>(local.width * local.height));
            if (fill_ratio < 0.20 || fill_ratio > 0.90) {
                continue;
            }

            const cv::Point2f center(
                static_cast<float>(roi.x + local.x + local.width * 0.5),
                static_cast<float>(roi.y + local.y + local.height * 0.5));
            const double distance = cv::norm(center - rois[corner].expected_center);
            const double score = area - distance * 18.0;
            if (score > best_score) {
                best_score = score;
                best_rect = cv::Rect(roi.x + local.x, roi.y + local.y, local.width, local.height);
            }
        }

        if (best_score < 0.0) {
            return false;
        }
        markers[corner] = best_rect;
    }

    return true;
}

bool derive_grid_from_markers(
    const cv::Mat& aligned,
    const EncoderConfig& cfg,
    double& origin_x,
    double& origin_y,
    double& cell_w,
    double& cell_h,
    int& rows) {
    std::array<cv::Rect, 4> markers{};
    if (!find_corner_markers(aligned, markers)) {
        return false;
    }

    const double marker_cell_w =
        (markers[0].width + markers[1].width + markers[2].width + markers[3].width) /
        (4.0 * FINDER_MARKER_CELLS);
    const double marker_cell_h =
        (markers[0].height + markers[1].height + markers[2].height + markers[3].height) /
        (4.0 * FINDER_MARKER_CELLS);

    origin_x = (markers[0].x + markers[2].x) * 0.5;
    origin_y = (markers[0].y + markers[1].y) * 0.5;

    const double right_edge =
        ((markers[1].x + markers[1].width) + (markers[3].x + markers[3].width)) * 0.5;
    const double bottom_edge =
        ((markers[2].y + markers[2].height) + (markers[3].y + markers[3].height)) * 0.5;
    const double total_w = right_edge - origin_x;
    const double total_h = bottom_edge - origin_y;
    if (total_w <= 1.0 || total_h <= 1.0) {
        return false;
    }

    const int total_cols = layout_total_cols(cfg);
    cell_w = total_w / std::max(1, total_cols);
    if (cell_w <= 1.0) {
        cell_w = marker_cell_w;
    }

    const double marker_cell = (marker_cell_w + marker_cell_h) * 0.5;
    rows = static_cast<int>(std::round(total_h / std::max(1.0, marker_cell)));
    rows = std::max(2 * FINDER_MARKER_CELLS + 2, rows);
    cell_h = total_h / rows;
    if (cell_h <= 1.0) {
        cell_h = marker_cell_h;
    }

    const bool sane =
        origin_x >= 0.0 &&
        origin_y >= 0.0 &&
        cell_w >= 4.0 &&
        cell_h >= 4.0 &&
        rows >= 1;
    return sane;
}

} // namespace

bool detect_frame_quad(
    const cv::Mat& frame,
    const EncoderConfig& cfg,
    std::array<cv::Point2f, 4>& out_quad) {
    (void)cfg;
    std::vector<FinderCandidate> candidates;
    std::array<FinderCandidate, 4> quad{};
    if (!detect_finder_candidates(frame, candidates) ||
        !select_finder_quad(candidates, quad) ||
        !quad_is_corner_aligned(quad, frame.size())) {
        return false;
    }

    out_quad = quad_points_from_finder_quad(quad);
    return true;
}

bool rectify_frame_with_quad(
    const cv::Mat& frame,
    const std::array<cv::Point2f, 4>& quad,
    const EncoderConfig& cfg,
    cv::Mat& out_rectified) {
    return warp_from_quad_points(frame, quad, cfg, out_rectified);
}

EncoderConfig make_default_encoder_config(int fps) {
    EncoderConfig cfg;
    cfg.fps = fps;
    cfg.cell_size = 15;     //后续好像跟这个cellsize没什么关系？？根据屏幕比例会缩放
    cfg.cells_per_row = 140;
    cfg.payload_bytes_per_frame = 3500;
    cfg.reference_block_size = 2;
    return cfg;
}

int required_total_rows_for_payload_cells(const EncoderConfig& cfg, int payload_cells) {
    return minimum_total_rows_for_cells(payload_cells, cfg);
}

void render_frame(
    cv::Mat& out,
    const std::vector<uint8_t>& payload,
    const EncoderConfig& cfg,
    int forced_rows) {
    std::vector<int> cells;
    cells.reserve(payload.size() * 4);
    for (const uint8_t byte : payload) {
        for (int i = 0; i < 4; ++i) {
            const int shift = 6 - 2 * i;
            cells.push_back((byte >> shift) & 0x3);
        }
    }

    const int total_cells = static_cast<int>(cells.size());
    const int total_rows =
        std::max(minimum_total_rows_for_cells(total_cells, cfg), std::max(1, forced_rows));
    const std::vector<LayoutCell> data_cells = build_data_cells(total_rows, cfg);
    const int marker_px = FINDER_MARKER_CELLS * cfg.cell_size;
    const int total_cols = layout_total_cols(cfg);
    const int img_w = total_cols * cfg.cell_size;
    const int img_h = total_rows * cfg.cell_size;

    out.create(img_h, img_w, CV_8UC3);
    out.setTo(cfg.background_color);

    auto draw_finder = [&](int x, int y) {
        cv::rectangle(out, cv::Rect(x, y, marker_px, marker_px), cv::Scalar(0, 0, 0), cv::FILLED);
        const int inner = cfg.cell_size;
        cv::rectangle(
            out,
            cv::Rect(x + inner, y + inner, marker_px - 2 * inner, marker_px - 2 * inner),
            cfg.background_color,
            cv::FILLED);
        const int inner2 = inner * 2;
        cv::rectangle(
            out,
            cv::Rect(x + inner2, y + inner2, marker_px - 2 * inner2, marker_px - 2 * inner2),
            cv::Scalar(0, 0, 0),
            cv::FILLED);
    };

    draw_finder(0, 0);
    draw_finder((total_cols - FINDER_MARKER_CELLS) * cfg.cell_size, 0);
    draw_finder(0, (total_rows - FINDER_MARKER_CELLS) * cfg.cell_size);
    draw_finder(
        (total_cols - FINDER_MARKER_CELLS) * cfg.cell_size,
        (total_rows - FINDER_MARKER_CELLS) * cfg.cell_size);

    const int small_marker_px = 2 * cfg.cell_size;
    auto draw_small = [&](int x, int y) {
        cv::rectangle(out, cv::Rect(x, y, small_marker_px, small_marker_px), cv::Scalar(0, 0, 0), cv::FILLED);
        const int inner = std::max(1, cfg.cell_size / 2);
        cv::rectangle(
            out,
            cv::Rect(x + inner, y + inner, small_marker_px - 2 * inner, small_marker_px - 2 * inner),
            cfg.background_color,
            cv::FILLED);
    };

    draw_small((img_w - small_marker_px) / 2, 0);
    draw_small((img_w - small_marker_px) / 2, img_h - small_marker_px);
    draw_small(0, (img_h - small_marker_px) / 2);
    draw_small(img_w - small_marker_px, (img_h - small_marker_px) / 2);

    for (int index = 0; index < total_cells && index < static_cast<int>(data_cells.size()); ++index) {
        const LayoutCell slot = data_cells[static_cast<size_t>(index)];
        cv::rectangle(
            out,
            cv::Rect(slot.col * cfg.cell_size, slot.row * cfg.cell_size, cfg.cell_size, cfg.cell_size),
            cfg.colors[cells[index]],
            cv::FILLED);
    }

    const int ref_px = cfg.reference_block_size * cfg.cell_size;
    const int ref_y = layout_ref_top_row() * cfg.cell_size;
    const int ref_x0 = layout_ref_left_col() * cfg.cell_size;
    for (int k = 0; k < 4; ++k) {
        const int ref_x = ref_x0 + k * ((cfg.reference_block_size + 1) * cfg.cell_size);
        cv::rectangle(out, cv::Rect(ref_x, ref_y, ref_px, ref_px), cfg.colors[k], cv::FILLED);
    }
}

bool rectify_frame_geometry(const cv::Mat& frame, cv::Mat& out_rectified, const EncoderConfig& cfg) {
    if (frame.empty()) {
        return false;
    }

    int expected_w = 0;
    int expected_h = 0;
    expected_canvas_size(cfg, expected_w, expected_h);
    const bool exact_raw_layout =
        frame.cols == expected_w &&
        frame.rows == expected_h &&
        has_raw_layout_corners(frame, cfg);
    if (exact_raw_layout) {
        out_rectified = frame.clone();
        return true;
    }

    if (warp_with_structured_finders(frame, out_rectified, cfg)) {
        return true;
    }
    if (warp_with_canvas_quad(frame, out_rectified, cfg)) {
        return true;
    }
    if (warp_with_finders(frame, out_rectified, cfg)) {
        return true;
    }
    return false;
}

bool sample_frame(const cv::Mat& frame, std::vector<uint8_t>& out_payload, EncoderConfig& cfg) {
    int expected_w = 0;
    int expected_h = 0;
    expected_canvas_size(cfg, expected_w, expected_h);
    const bool exact_raw_layout =
        frame.cols == expected_w &&
        frame.rows == expected_h &&
        has_raw_layout_corners(frame, cfg);

    auto sample_aligned = [&](const cv::Mat& aligned, const char* label) -> bool {
        const int img_w = aligned.cols;
        const int img_h = aligned.rows;
        const std::string_view mode(label);

        double origin_x = 0.0;
        double origin_y = 0.0;
        double cell_w = cfg.cell_size;
        double cell_h = cfg.cell_size;
        int total_rows = 0;
        const bool marker_layout =
            (mode == "cropped" || mode == "warped") &&
            derive_grid_from_markers(aligned, cfg, origin_x, origin_y, cell_w, cell_h, total_rows);

        if (marker_layout) {
            // Use geometry inferred from the four large finder blocks when available.
        } else if (mode == "warped") {
            cell_w = static_cast<double>(img_w) / std::max(1, layout_total_cols(cfg));
            cell_h = cell_w;
            origin_x = 0.0;
            origin_y = 0.0;
            total_rows = static_cast<int>(std::round(img_h / std::max(1.0, cell_h)));
        } else if ((mode == "raw" || mode == "aligned") && img_w == expected_w && img_h == expected_h) {
            origin_x = 0.0;
            origin_y = 0.0;
            total_rows = img_h / cfg.cell_size;
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

            origin_x = static_cast<double>(std::max(0, col_lo - (col_lo % cfg.cell_size)));
            origin_y = static_cast<double>(std::max(0, row_lo - (row_lo % cfg.cell_size)));
            const int data_w = img_w;
            if (origin_x + data_w > img_w) {
                origin_x = static_cast<double>(std::max(0, img_w - data_w));
            }
            total_rows = std::max(2 * FINDER_MARKER_CELLS + 2, (row_hi - static_cast<int>(origin_y) + 1) / cfg.cell_size);
        }

        if (total_rows <= 0) {
            return false;
        }

        cv::Mat sample_source;
        cv::GaussianBlur(aligned, sample_source, cv::Size(3, 3), 0.0);

        std::array<cv::Scalar, 4> classifier_colors = {
            cfg.colors[0], cfg.colors[1], cfg.colors[2], cfg.colors[3],
        };

        const double ref_cells = static_cast<double>(cfg.reference_block_size);
        const double ref_w = std::max(1.0, ref_cells * cell_w);
        const double ref_h = std::max(1.0, ref_cells * cell_h);
        const double ref_gap = cell_w;
        const double ref_y = origin_y + layout_ref_top_row() * cell_h;
        const double ref_x0 = origin_x + layout_ref_left_col() * cell_w;
        if (ref_y >= 0.0) {
            std::array<cv::Scalar, 4> observed_refs{};
            bool refs_ok = true;

            for (int k = 0; k < 4; ++k) {
                const double ref_x = ref_x0 + k * (ref_w + ref_gap);
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
                observed_refs[k] = cv::mean(sample_source(cv::Rect(
                    x0 + inset_x,
                    y0 + inset_y,
                    std::max(1, rw - 2 * inset_x),
                    std::max(1, rh - 2 * inset_y))));
            }

            if (refs_ok) {
                classifier_colors = observed_refs;
            }
        }

        auto build_payload_candidates = [&](double ox, double oy, double cw, double ch, int total_rows_local) {
            const std::vector<LayoutCell> sample_cells = build_data_cells(total_rows_local, cfg);
            std::vector<int> cells_local_chroma;
            std::vector<int> cells_local_missing;
            cells_local_chroma.reserve(sample_cells.size());
            cells_local_missing.reserve(sample_cells.size());

            const cv::Vec3d black_ref(
                classifier_colors[0][0],
                classifier_colors[0][1],
                classifier_colors[0][2]);
            std::array<cv::Vec3d, 4> ref_chroma{};
            std::array<int, 3> missing_channel_to_symbol = {-1, -1, -1};
            double min_color_energy = std::numeric_limits<double>::max();
            for (int k = 1; k < 4; ++k) {
                cv::Vec3d ref(
                    std::max(0.0, classifier_colors[k][0] - black_ref[0]),
                    std::max(0.0, classifier_colors[k][1] - black_ref[1]),
                    std::max(0.0, classifier_colors[k][2] - black_ref[2]));
                const double energy = ref[0] + ref[1] + ref[2];
                min_color_energy = std::min(min_color_energy, energy);
                if (energy > 1e-6) {
                    ref /= energy;
                }
                ref_chroma[k] = ref;

                int missing_channel = 0;
                if (ref[1] < ref[missing_channel]) {
                    missing_channel = 1;
                }
                if (ref[2] < ref[missing_channel]) {
                    missing_channel = 2;
                }
                missing_channel_to_symbol[missing_channel] = k;
            }
            const double black_threshold = std::max(18.0, min_color_energy * 0.20);

            for (const LayoutCell& slot : sample_cells) {
                    const int x = static_cast<int>(std::round(ox + slot.col * cw));
                    const int y = static_cast<int>(std::round(oy + slot.row * ch));
                    const int cell_px_w = static_cast<int>(std::round(cw));
                    const int cell_px_h = static_cast<int>(std::round(ch));
                    if (x < 0 || y < 0 || x + cell_px_w > img_w || y + cell_px_h > img_h) {
                        return std::vector<std::vector<uint8_t>>{};
                    }

                    const int inset_x = std::max(1, cell_px_w / 3);
                    const int inset_y = std::max(1, cell_px_h / 3);
                    const cv::Rect cell_rect(
                        x + inset_x,
                        y + inset_y,
                        std::max(1, cell_px_w - 2 * inset_x),
                        std::max(1, cell_px_h - 2 * inset_y));
                    const cv::Scalar mean = cv::mean(sample_source(cell_rect));

                    cv::Vec3d cell(
                        std::max(0.0, mean[0] - black_ref[0]),
                        std::max(0.0, mean[1] - black_ref[1]),
                        std::max(0.0, mean[2] - black_ref[2]));
                    const double energy = cell[0] + cell[1] + cell[2];

                    int chroma_symbol = 0;
                    int missing_symbol = 0;
                    if (energy <= black_threshold) {
                        chroma_symbol = 0;
                        missing_symbol = 0;
                    } else {
                        int missing_channel = 0;
                        if (cell[1] < cell[missing_channel]) {
                            missing_channel = 1;
                        }
                        if (cell[2] < cell[missing_channel]) {
                            missing_channel = 2;
                        }
                        missing_symbol = missing_channel_to_symbol[missing_channel];
                        if (missing_symbol < 1 || missing_symbol > 3) {
                            missing_symbol = 0;
                        }

                        cell /= energy;
                        double bestd = std::numeric_limits<double>::infinity();
                        for (int k = 1; k < 4; ++k) {
                            const cv::Vec3d diff = cell - ref_chroma[k];
                            const double d = diff.dot(diff);
                            if (d < bestd) {
                                chroma_symbol = k;
                                bestd = d;
                            }
                        }
                    }
                    cells_local_chroma.push_back(chroma_symbol);
                    cells_local_missing.push_back(missing_symbol);
            }

            std::vector<std::vector<uint8_t>> payloads;
            payloads.reserve(8);
            auto append_payloads = [&](const std::vector<int>& cells_local) {
                for (int phase = 0; phase < 4; ++phase) {
                    if (cells_local.size() <= static_cast<size_t>(phase)) {
                        break;
                    }
                    std::vector<uint8_t> payload;
                    payload.reserve((cells_local.size() - static_cast<size_t>(phase)) / 4);
                    for (size_t i = static_cast<size_t>(phase); i + 3 < cells_local.size(); i += 4) {
                        uint8_t byte = 0;
                        for (int j = 0; j < 4; ++j) {
                            byte |= static_cast<uint8_t>((cells_local[i + j] & 0x3) << (6 - 2 * j));
                        }
                        payload.push_back(byte);
                    }
                    if (!payload.empty()) {
                        payloads.push_back(std::move(payload));
                    }
                }
            };

            append_payloads(cells_local_chroma);
            if (cells_local_missing != cells_local_chroma) {
                append_payloads(cells_local_missing);
            }
            return payloads;
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
        if (marker_layout) {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.05, 0.0, 1.0, 1.0, 0}, {0.05, 0.0, 1.0, 1.0, 0},
                {0.0, -0.05, 1.0, 1.0, 0}, {0.0, 0.05, 1.0, 1.0, 0},
                {0.0, 0.0, 0.995, 0.995, 0}, {0.0, 0.0, 1.005, 1.005, 0},
                {0.0, 0.0, 1.0, 1.0, -1}, {0.0, 0.0, 1.0, 1.0, 1},
            };
        } else if (mode == "warped") {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.15, 0.0, 1.0, 1.0, 0}, {0.15, 0.0, 1.0, 1.0, 0},
                {0.0, -0.15, 1.0, 1.0, 0}, {0.0, 0.15, 1.0, 1.0, 0},
                {-0.10, -0.10, 1.0, 1.0, 0}, {0.10, 0.10, 1.0, 1.0, 0},
                {0.0, 0.0, 0.985, 0.985, 0}, {0.0, 0.0, 1.015, 1.015, 0},
                {0.0, 0.0, 0.97, 0.97, 0}, {0.0, 0.0, 1.03, 1.03, 0},
                {0.0, 0.0, 1.0, 0.97, 0}, {0.0, 0.0, 1.0, 1.03, 0},
                {0.0, 0.0, 1.0, 1.0, -2}, {0.0, 0.0, 1.0, 1.0, -1},
                {0.0, 0.0, 1.0, 1.0, 1}, {0.0, 0.0, 1.0, 1.0, 2},
                {0.0, 0.0, 0.985, 0.985, -2}, {0.0, 0.0, 0.985, 0.985, -1},
                {0.0, 0.0, 0.985, 0.985, 1}, {0.0, 0.0, 0.985, 0.985, 2},
                {0.0, 0.0, 0.97, 0.97, -2}, {0.0, 0.0, 0.97, 0.97, -1},
                {0.0, 0.0, 0.97, 0.97, 1}, {0.0, 0.0, 0.97, 0.97, 2},
            };
        } else if (mode == "aligned") {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.08, 0.0, 1.0, 1.0, 0}, {0.08, 0.0, 1.0, 1.0, 0},
                {0.0, -0.08, 1.0, 1.0, 0}, {0.0, 0.08, 1.0, 1.0, 0},
                {0.0, 0.0, 0.99, 0.99, 0}, {0.0, 0.0, 1.01, 1.01, 0},
                {0.0, 0.0, 0.985, 0.985, 0}, {0.0, 0.0, 1.015, 1.015, 0},
                {0.0, 0.0, 1.0, 1.0, -1}, {0.0, 0.0, 1.0, 1.0, 1},
                {0.0, 0.0, 0.99, 0.99, -1}, {0.0, 0.0, 0.99, 0.99, 1},
                {0.0, 0.0, 1.01, 1.01, -1}, {0.0, 0.0, 1.01, 1.01, 1},
            };
        } else if (mode == "cropped") {
            variants = {
                {0.0, 0.0, 1.0, 1.0, 0},
                {-0.15, 0.0, 1.0, 1.0, 0}, {0.15, 0.0, 1.0, 1.0, 0},
                {0.0, -0.15, 1.0, 1.0, 0}, {0.0, 0.15, 1.0, 1.0, 0},
                {0.0, 0.0, 0.985, 0.985, 0}, {0.0, 0.0, 1.015, 1.015, 0},
                {0.0, 0.0, 0.97, 0.97, 0}, {0.0, 0.0, 1.03, 1.03, 0},
                {0.0, 0.0, 1.0, 1.0, -2}, {0.0, 0.0, 1.0, 1.0, -1},
                {0.0, 0.0, 1.0, 1.0, 1}, {0.0, 0.0, 1.0, 1.0, 2},
            };
        }

        int best_score = std::numeric_limits<int>::max();
        bool best_crc_match = false;
        std::vector<uint8_t> best_payload;
        for (const auto& variant : variants) {
            const int trial_rows = total_rows + variant.row_delta;
            if (trial_rows <= 2 * FINDER_MARKER_CELLS + 1) {
                continue;
            }

            std::vector<std::vector<uint8_t>> payload_candidates = build_payload_candidates(
                origin_x + variant.dx * cell_w,
                origin_y + variant.dy * cell_h,
                cell_w * variant.sx,
                cell_h * variant.sy,
                trial_rows);
            if (payload_candidates.empty()) {
                continue;
            }

            for (auto& payload : payload_candidates) {
                const int score = header_distance(payload);
                bool crc_match = false;
                if (payload.size() >= 21 &&
                    payload[0] == 0x43 &&
                    payload[1] == 0x4D &&
                    payload[2] == 0x41 &&
                    payload[3] == 0x43 &&
                    payload[4] == 0x01) {
                    const uint32_t frame_index =
                        static_cast<uint32_t>(payload[5]) |
                        (static_cast<uint32_t>(payload[6]) << 8) |
                        (static_cast<uint32_t>(payload[7]) << 16) |
                        (static_cast<uint32_t>(payload[8]) << 24);
                    const uint32_t total_frames =
                        static_cast<uint32_t>(payload[9]) |
                        (static_cast<uint32_t>(payload[10]) << 8) |
                        (static_cast<uint32_t>(payload[11]) << 16) |
                        (static_cast<uint32_t>(payload[12]) << 24);
                    const uint32_t payload_bytes =
                        static_cast<uint32_t>(payload[13]) |
                        (static_cast<uint32_t>(payload[14]) << 8) |
                        (static_cast<uint32_t>(payload[15]) << 16) |
                        (static_cast<uint32_t>(payload[16]) << 24);
                    const uint32_t checksum =
                        static_cast<uint32_t>(payload[17]) |
                        (static_cast<uint32_t>(payload[18]) << 8) |
                        (static_cast<uint32_t>(payload[19]) << 16) |
                        (static_cast<uint32_t>(payload[20]) << 24);

                    if (total_frames > 0 &&
                        frame_index < total_frames &&
                        payload_bytes > 0 &&
                        payload_bytes <= static_cast<uint32_t>(cfg.payload_bytes_per_frame) &&
                        21u + payload_bytes <= payload.size()) {
                        crc_match = crc32(payload.data() + 21, payload_bytes) == checksum;
                    }
                }

                if ((crc_match && !best_crc_match) ||
                    (crc_match == best_crc_match &&
                     (score < best_score ||
                      (score == best_score && payload.size() > best_payload.size())))) {
                    best_crc_match = crc_match;
                    best_score = score;
                    best_payload = std::move(payload);
                    if (crc_match) {
                        break;
                    }
                }
            }
            if (best_crc_match) {
                break;
            }
        }

        if (best_payload.empty()) {
            return false;
        }

        out_payload = std::move(best_payload);
        return true;
    };

    if (exact_raw_layout && sample_aligned(frame, "raw")) {
        return true;
    }

    cv::Mat finder_warped;
    if (warp_with_structured_finders(frame, finder_warped, cfg) && sample_aligned(finder_warped, "aligned")) {
        return true;
    }

    cv::Mat canvas_warped;
    if (warp_with_canvas_quad(frame, canvas_warped, cfg) && sample_aligned(canvas_warped, "aligned")) {
        return true;
    }

    cv::Mat cropped = center_crop_expected_layout(frame, cfg);
    if (sample_aligned(cropped, "cropped")) {
        return true;
    }

    cv::Mat warped;
    if (warp_with_finders(frame, warped, cfg) && sample_aligned(warped, "warped")) {
        return true;
    }
    return false;
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
