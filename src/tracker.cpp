#include "tracker.hpp"

namespace camcom {

CornerKalman::CornerKalman() : kf_(4,2,0) {
    // state: x, y, vx, vy
    // measurement: x, y
    kf_.transitionMatrix = (cv::Mat_<float>(4,4) <<
        1,0,1,0,
        0,1,0,1,
        0,0,1,0,
        0,0,0,1);
    kf_.measurementMatrix = cv::Mat::zeros(2,4,CV_32F);
    kf_.measurementMatrix.at<float>(0,0) = 1.0f;
    kf_.measurementMatrix.at<float>(1,1) = 1.0f;
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-3));
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
}

void CornerKalman::init(const cv::Point2f& pt) {
    kf_.statePost.at<float>(0) = pt.x;
    kf_.statePost.at<float>(1) = pt.y;
    kf_.statePost.at<float>(2) = 0;
    kf_.statePost.at<float>(3) = 0;
    initialized_ = true;
}

cv::Point2f CornerKalman::predict() {
    cv::Mat pred = kf_.predict();
    return cv::Point2f(pred.at<float>(0), pred.at<float>(1));
}

cv::Point2f CornerKalman::correct(const cv::Point2f& meas) {
    cv::Mat measurement(2,1,CV_32F);
    measurement.at<float>(0) = meas.x;
    measurement.at<float>(1) = meas.y;
    cv::Mat state = kf_.correct(measurement);
    return cv::Point2f(state.at<float>(0), state.at<float>(1));
}

// QuadTracker
QuadTracker::QuadTracker() {}

void QuadTracker::init(const std::array<cv::Point2f,4>& pts) {
    for (int i = 0; i < 4; ++i) {
        corners_[i].init(pts[i]);
        last_[i] = pts[i];
    }
    initialized_ = true;
}

void QuadTracker::update(const std::array<cv::Point2f,4>& meas) {
    if (!initialized_) {
        init(meas);
        return;
    }
    for (int i = 0; i < 4; ++i) {
        if (meas[i].x >= 0 && meas[i].y >= 0) {
            last_[i] = corners_[i].correct(meas[i]);
        } else {
            last_[i] = corners_[i].predict();
        }
    }
}

std::array<cv::Point2f,4> QuadTracker::get() const {
    return last_;
}

} // namespace camcom
