#pragma once

#include <opencv2/opencv.hpp>
#include <array>

namespace camcom {

// Simple Kalman-based tracker for four corner points. Each corner has a 4-state Kalman filter (x,y,dx,dy)
class CornerKalman {
public:
    CornerKalman();
    void init(const cv::Point2f& pt);
    cv::Point2f predict();
    cv::Point2f correct(const cv::Point2f& meas);
private:
    cv::KalmanFilter kf_; // 4 state, 2 measure
    bool initialized_ = false;
};

class QuadTracker {
public:
    QuadTracker();
    void init(const std::array<cv::Point2f,4>& pts);
    void update(const std::array<cv::Point2f,4>& meas);
    std::array<cv::Point2f,4> get() const;
    bool is_initialized() const { return initialized_; }
private:
    std::array<CornerKalman,4> corners_;
    std::array<cv::Point2f,4> last_; 
    bool initialized_ = false;
};

} // namespace camcom
