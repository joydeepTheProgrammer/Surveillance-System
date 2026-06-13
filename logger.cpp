#include "surveillance.hpp"

Logger::Logger(const std::string& filename) {
    log_file.open(filename, std::ios::app);
    writer_thread = std::thread(&Logger::writerLoop, this);
}

Logger::~Logger() {
    running = false;
    cv.notify_all();
    if (writer_thread.joinable()) writer_thread.join();
    if (log_file.is_open()) log_file.close();
}

void Logger::log(const std::string& level, const std::string& msg) {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << " [" << level << "] " << msg;

    std::lock_guard<std::mutex> lock(mtx);
    buffer.push(ss.str());
    cv.notify_one();
}

void Logger::writerLoop() {
    while (running || !buffer.empty()) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !buffer.empty() || !running; });

        while (!buffer.empty()) {
            auto msg = buffer.front();
            buffer.pop();
            lock.unlock();

            std::cout << msg << std::endl;
            if (log_file.is_open()) {
                log_file << msg << std::endl;
                log_file.flush();
            }
            lock.lock();
        }
    }
}
