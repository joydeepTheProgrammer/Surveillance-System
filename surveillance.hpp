#ifndef SURVEILLANCE_HPP
#define SURVEILLANCE_HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <vector>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cstring>
#include <csignal>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <gpiod.h>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>

// ============================================================================
// CONFIGURATION
// ============================================================================

namespace config {
    // Camera
    constexpr int CAM_WIDTH = 1280;
    constexpr int CAM_HEIGHT = 720;
    constexpr int CAM_FPS = 30;
    constexpr const char* CAM_DEVICE = "/dev/video0";

    // Motion Detection
    constexpr int MOTION_THRESHOLD = 25;
    constexpr int MOTION_MIN_AREA = 500;
    constexpr int MOTION_HISTORY = 500;
    constexpr int MOTION_VAR_THRESHOLD = 16;
    constexpr int MOTION_COOLDOWN_SEC = 5;
    constexpr int RECORD_DURATION_SEC = 15;

    // Recording
    constexpr const char* RECORD_DIR = "/home/pi/surveillance/recordings";
    constexpr int MAX_RECORD_AGE_DAYS = 30;
    constexpr const char* VIDEO_CODEC = "mp4v";
    constexpr const char* VIDEO_EXT = ".mp4";

    // Streaming
    constexpr int STREAM_PORT = 8080;
    constexpr int STREAM_QUALITY = 80;
    constexpr int STREAM_FPS = 30;
    constexpr int MAX_CLIENTS = 10;

    // GPIO (BCM numbering)
    constexpr int PIR_PIN = 17;
    constexpr int IR_LED_PIN = 18;
    constexpr int BUZZER_PIN = 27;
    constexpr int STATUS_LED_PIN = 22;

    // Night Vision
    constexpr int NIGHT_THRESHOLD = 50;
    constexpr int NIGHT_HYSTERESIS = 20;

    // System
    constexpr int BUFFER_POOL_SIZE = 4;
    constexpr int LOG_BUFFER_SIZE = 4096;
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct FrameBuffer {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
    std::atomic<bool> in_use{false};
    std::atomic<bool> has_motion{false};
    std::vector<cv::Rect> motion_regions;
    double brightness = 0.0;
    int frame_number = 0;
};

struct SystemStatus {
    std::atomic<bool> running{false};
    std::atomic<bool> motion_detected{false};
    std::atomic<bool> is_recording{false};
    std::atomic<bool> night_mode{false};
    std::atomic<bool> pir_triggered{false};
    std::atomic<int> fps{0};
    std::atomic<int> total_frames{0};
    std::atomic<int> motion_events{0};
    std::atomic<int> recording_count{0};
    std::string current_recording_file;
    std::mutex status_mutex;
};

// ============================================================================
// LOGGER (Thread-safe ring buffer)
// ============================================================================

class Logger {
private:
    std::ofstream log_file;
    std::mutex mtx;
    std::queue<std::string> buffer;
    std::condition_variable cv;
    std::thread writer_thread;
    std::atomic<bool> running{true};

    void writerLoop();

public:
    Logger(const std::string& filename);
    ~Logger();

    void log(const std::string& level, const std::string& msg);
    void info(const std::string& msg) { log("INFO", msg); }
    void warn(const std::string& msg) { log("WARN", msg); }
    void error(const std::string& msg) { log("ERROR", msg); }
    void debug(const std::string& msg) { log("DEBUG", msg); }
};

// ============================================================================
// GPIO MANAGER (libgpiod - modern replacement for wiringPi)
// ============================================================================

class GPIOManager {
private:
    struct gpiod_chip* chip = nullptr;
    struct gpiod_line* pir_line = nullptr;
    struct gpiod_line* ir_line = nullptr;
    struct gpiod_line* buzzer_line = nullptr;
    struct gpiod_line* status_line = nullptr;
    std::thread monitor_thread;
    std::atomic<bool> running{false};
    SystemStatus* status = nullptr;
    Logger* logger = nullptr;

    void monitorLoop();

public:
    GPIOManager(SystemStatus* s, Logger* l);
    ~GPIOManager();

    bool init();
    void start();
    void stop();

    void setIR(bool state);
    void setBuzzer(bool state);
    void setStatusLED(bool state);
    void pulseBuzzer(int ms);
    bool readPIR();
};

// ============================================================================
// V4L2 CAMERA CAPTURE (Zero-copy DMA)
// ============================================================================

class V4L2Camera {
private:
    int fd = -1;
    struct v4l2_buffer buf;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;

    struct Buffer {
        void* start;
        size_t length;
    };
    std::vector<Buffer> buffers;

    std::thread capture_thread;
    std::atomic<bool> running{false};
    SystemStatus* status = nullptr;
    Logger* logger = nullptr;

    FrameBuffer* pool = nullptr;
    int pool_size = 0;
    std::atomic<int> write_idx{0};
    std::atomic<int> read_idx{0};

    bool initDevice();
    bool initBuffers();
    void captureLoop();
    void convertYUYVtoBGR(const uint8_t* yuyv, cv::Mat& bgr, int width, int height);

public:
    V4L2Camera(SystemStatus* s, Logger* l, FrameBuffer* p, int pool_size);
    ~V4L2Camera();

    bool start();
    void stop();
    FrameBuffer* acquireFrame();
    void releaseFrame(FrameBuffer* fb);
};

// ============================================================================
// MOTION DETECTOR (Background subtraction + contour analysis)
// ============================================================================

class MotionDetector {
private:
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg_subtractor;
    cv::Mat kernel;
    SystemStatus* status = nullptr;
    Logger* logger = nullptr;

    std::chrono::steady_clock::time_point last_motion_time;
    std::atomic<bool> active{false};
    std::mutex mtx;

public:
    MotionDetector(SystemStatus* s, Logger* l);

    bool detect(FrameBuffer* fb);
    void drawRegions(cv::Mat& frame, const std::vector<cv::Rect>& regions);
};

// ============================================================================
// VIDEO RECORDER (H.264 hardware encoding via OpenCV)
// ============================================================================

class VideoRecorder {
private:
    cv::VideoWriter writer;
    std::string current_file;
    std::chrono::steady_clock::time_point record_start;
    std::mutex mtx;
    Logger* logger = nullptr;
    SystemStatus* status = nullptr;

    std::string generateFilename();

public:
    VideoRecorder(SystemStatus* s, Logger* l);

    bool startRecording(const cv::Size& size, double fps);
    void writeFrame(const cv::Mat& frame);
    void stopRecording();
    bool shouldStop();
    bool isRecording();
    void cleanupOldRecordings();
};

// ============================================================================
// HTTP STREAMING SERVER (Multi-threaded MJPEG)
// ============================================================================

class StreamServer {
private:
    int server_fd = -1;
    int port;
    std::thread server_thread;
    std::vector<std::thread> client_threads;
    std::atomic<bool> running{false};
    SystemStatus* status = nullptr;
    Logger* logger = nullptr;

    FrameBuffer* current_frame = nullptr;
    std::mutex frame_mutex;

    void serverLoop();
    void handleClient(int client_fd);
    bool sendMJPEG(int client_fd, const cv::Mat& frame);

public:
    StreamServer(int p, SystemStatus* s, Logger* l);
    ~StreamServer();

    bool start();
    void stop();
    void updateFrame(FrameBuffer* fb);
};

// ============================================================================
// ALERT MANAGER (Telegram, Email, Buzzer)
// ============================================================================

class AlertManager {
private:
    std::thread alert_thread;
    std::queue<std::string> alert_queue;
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<bool> running{false};
    GPIOManager* gpio = nullptr;
    Logger* logger = nullptr;

    void alertLoop();
    void sendTelegram(const std::string& msg);
    void sendEmail(const std::string& msg);

public:
    AlertManager(GPIOManager* g, Logger* l);

    void start();
    void stop();
    void trigger(const std::string& type, const std::string& details);
};

// ============================================================================
// MAIN SURVEILLANCE SYSTEM
// ============================================================================

class SurveillanceSystem {
private:
    SystemStatus status;
    Logger* logger = nullptr;
    GPIOManager* gpio = nullptr;
    V4L2Camera* camera = nullptr;
    MotionDetector* detector = nullptr;
    VideoRecorder* recorder = nullptr;
    StreamServer* streamer = nullptr;
    AlertManager* alerter = nullptr;

    FrameBuffer* buffer_pool = nullptr;

    std::thread processing_thread;
    std::thread recording_thread;
    std::atomic<bool> running{false};

    void processingLoop();
    void recordingLoop();

public:
    SurveillanceSystem();
    ~SurveillanceSystem();

    bool init();
    void start();
    void stop();
    SystemStatus* getStatus() { return &status; }
};

#endif // SURVEILLANCE_HPP
