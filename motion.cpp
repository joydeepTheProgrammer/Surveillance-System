#include "surveillance.hpp"

MotionDetector::MotionDetector(SystemStatus* s, Logger* l) 
    : status(s), logger(l) {

    bg_subtractor = cv::createBackgroundSubtractorMOG2(
        config::MOTION_HISTORY,
        config::MOTION_VAR_THRESHOLD,
        false
    );
    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    last_motion_time = std::chrono::steady_clock::now();
}

bool MotionDetector::detect(FrameBuffer* fb) {
    if (fb->frame.empty()) return false;

    cv::Mat fg_mask;
    bg_subtractor->apply(fb->frame, fg_mask);

    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);

    cv::threshold(fg_mask, fg_mask, config::MOTION_THRESHOLD, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_motion_time).count();

    bool significant = false;
    fb->motion_regions.clear();

    for (auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area > config::MOTION_MIN_AREA) {
            fb->motion_regions.push_back(cv::boundingRect(cnt));
            significant = true;
        }
    }

    if (significant && elapsed > config::MOTION_COOLDOWN_SEC) {
        last_motion_time = now;
        active = true;
        status->motion_detected = true;
        fb->has_motion = true;

        std::lock_guard<std::mutex> lock(mtx);
        status->motion_events++;

        logger->info("Motion detected! Regions: " + std::to_string(fb->motion_regions.size()));
        return true;
    }

    if (elapsed > 2) {
        active = false;
        status->motion_detected = false;
    }

    return false;
}

void MotionDetector::drawRegions(cv::Mat& frame, const std::vector<cv::Rect>& regions) {
    for (auto& r : regions) {
        cv::rectangle(frame, r, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, "MOTION", cv::Point(r.x, r.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
}
