#include "surveillance.hpp"

SurveillanceSystem::SurveillanceSystem() {}

SurveillanceSystem::~SurveillanceSystem() {
    stop();
}

bool SurveillanceSystem::init() {
    status.running = true;

    logger = new Logger("/home/pi/surveillance/logs/system.log");
    logger->info("=== Surveillance System Starting ===");

    buffer_pool = new FrameBuffer[config::BUFFER_POOL_SIZE];
    for (int i = 0; i < config::BUFFER_POOL_SIZE; i++) {
        buffer_pool[i].in_use = false;
    }

    gpio = new GPIOManager(&status, logger);
    if (!gpio->init()) {
        logger->error("GPIO init failed");
        return false;
    }

    camera = new V4L2Camera(&status, logger, buffer_pool, config::BUFFER_POOL_SIZE);
    detector = new MotionDetector(&status, logger);
    recorder = new VideoRecorder(&status, logger);
    streamer = new StreamServer(config::STREAM_PORT, &status, logger);
    alerter = new AlertManager(gpio, logger);

    if (!camera->start()) {
        logger->error("Camera init failed");
        return false;
    }

    if (!streamer->start()) {
        logger->error("Stream server init failed");
        return false;
    }

    gpio->start();
    alerter->start();

    processing_thread = std::thread(&SurveillanceSystem::processingLoop, this);
    recording_thread = std::thread(&SurveillanceSystem::recordingLoop, this);

    logger->info("System initialized successfully");
    logger->info("Stream available at: http://<pi-ip>:" + std::to_string(config::STREAM_PORT));

    return true;
}

void SurveillanceSystem::processingLoop() {
    while (status.running) {
        FrameBuffer* fb = camera->acquireFrame();
        if (!fb) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        if (fb->brightness < config::NIGHT_THRESHOLD && !status.night_mode.load()) {
            gpio->setIR(true);
        } else if (fb->brightness > config::NIGHT_THRESHOLD + config::NIGHT_HYSTERESIS 
                   && status.night_mode.load()) {
            gpio->setIR(false);
        }

        bool motion = detector->detect(fb);

        if (motion) {
            alerter->trigger("MOTION", "Frame " + std::to_string(fb->frame_number));

            if (!recorder->isRecording()) {
                recorder->startRecording(fb->frame.size(), config::CAM_FPS);
            }
        }

        if (!fb->motion_regions.empty()) {
            detector->drawRegions(fb->frame, fb->motion_regions);
        }

        streamer->updateFrame(fb);

        static auto last_fps_time = std::chrono::steady_clock::now();
        static int frame_count = 0;
        frame_count++;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_time).count();
        if (elapsed >= 1) {
            status.fps = frame_count / elapsed;
            frame_count = 0;
            last_fps_time = now;
        }

        camera->releaseFrame(fb);
    }
}

void SurveillanceSystem::recordingLoop() {
    while (status.running) {
        FrameBuffer* fb = nullptr;
        for (int i = 0; i < config::BUFFER_POOL_SIZE; i++) {
            if (buffer_pool[i].in_use.load() && buffer_pool[i].has_motion.load()) {
                fb = &buffer_pool[i];
                break;
            }
        }

        if (fb && recorder->isRecording()) {
            recorder->writeFrame(fb->frame);
        }

        if (recorder->shouldStop()) {
            recorder->stopRecording();
            recorder->cleanupOldRecordings();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

void SurveillanceSystem::start() {
    if (!init()) {
        std::cerr << "Failed to initialize system" << std::endl;
        return;
    }

    while (status.running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void SurveillanceSystem::stop() {
    status.running = false;

    if (processing_thread.joinable()) processing_thread.join();
    if (recording_thread.joinable()) recording_thread.join();

    if (recorder) recorder->stopRecording();
    if (alerter) alerter->stop();
    if (gpio) gpio->stop();
    if (streamer) streamer->stop();
    if (camera) camera->stop();

    delete[] buffer_pool;
    delete logger;
    delete gpio;
    delete camera;
    delete detector;
    delete recorder;
    delete streamer;
    delete alerter;
}

SurveillanceSystem* g_system = nullptr;

void signal_handler(int sig) {
    std::cout << "
Received signal " << sig << ", shutting down..." << std::endl;
    if (g_system) g_system->stop();
}

int main(int argc, char** argv) {
    std::cout << "Home Surveillance System v2.0 (C++ High Performance)" << std::endl;
    std::cout << "=====================================================" << std::endl;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    SurveillanceSystem sys;
    g_system = &sys;

    sys.start();

    return 0;
}
