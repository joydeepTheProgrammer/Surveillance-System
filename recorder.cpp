#include "surveillance.hpp"
#include <dirent.h>

VideoRecorder::VideoRecorder(SystemStatus* s, Logger* l) : status(s), logger(l) {
    struct stat st;
    if (stat(config::RECORD_DIR, &st) != 0) {
        mkdir(config::RECORD_DIR, 0755);
    }
}

std::string VideoRecorder::generateFilename() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << config::RECORD_DIR << "/motion_";
    ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
    ss << config::VIDEO_EXT;

    return ss.str();
}

bool VideoRecorder::startRecording(const cv::Size& size, double fps) {
    std::lock_guard<std::mutex> lock(mtx);

    if (isRecording()) return false;

    current_file = generateFilename();
    int fourcc = cv::VideoWriter::fourcc(
        config::VIDEO_CODEC[0],
        config::VIDEO_CODEC[1],
        config::VIDEO_CODEC[2],
        config::VIDEO_CODEC[3]
    );

    writer.open(current_file, fourcc, fps, size, true);

    if (!writer.isOpened()) {
        logger->error("Failed to open video writer: " + current_file);
        return false;
    }

    record_start = std::chrono::steady_clock::now();
    status->is_recording = true;
    status->current_recording_file = current_file;
    status->recording_count++;

    logger->info("Recording started: " + current_file);
    return true;
}

void VideoRecorder::writeFrame(const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(mtx);
    if (writer.isOpened()) {
        cv::Mat annotated = frame.clone();

        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ts;
        ts << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");

        cv::putText(annotated, ts.str(), cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(annotated, "● REC", cv::Point(annotated.cols - 120, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        writer.write(annotated);
    }
}

void VideoRecorder::stopRecording() {
    std::lock_guard<std::mutex> lock(mtx);
    if (writer.isOpened()) {
        writer.release();
        status->is_recording = false;
        logger->info("Recording stopped: " + current_file);
        current_file.clear();
    }
}

bool VideoRecorder::shouldStop() {
    if (!isRecording()) return false;
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - record_start).count();
    return elapsed >= config::RECORD_DURATION_SEC;
}

bool VideoRecorder::isRecording() {
    return writer.isOpened();
}

void VideoRecorder::cleanupOldRecordings() {
    DIR* dir = opendir(config::RECORD_DIR);
    if (!dir) return;

    auto now = std::chrono::system_clock::now();
    auto cutoff = now - std::chrono::hours(24 * config::MAX_RECORD_AGE_DAYS);

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type != DT_REG) continue;

        std::string filepath = std::string(config::RECORD_DIR) + "/" + entry->d_name;
        struct stat st;
        if (stat(filepath.c_str(), &st) == 0) {
            auto mtime = std::chrono::system_clock::from_time_t(st.st_mtime);
            if (mtime < cutoff) {
                unlink(filepath.c_str());
                logger->info("Deleted old recording: " + std::string(entry->d_name));
            }
        }
    }
    closedir(dir);
}
