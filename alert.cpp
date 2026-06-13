#include "surveillance.hpp"

AlertManager::AlertManager(GPIOManager* g, Logger* l) : gpio(g), logger(l) {}

void AlertManager::start() {
    running = true;
    alert_thread = std::thread(&AlertManager::alertLoop, this);
}

void AlertManager::stop() {
    running = false;
    cv.notify_all();
    if (alert_thread.joinable()) alert_thread.join();
}

void AlertManager::trigger(const std::string& type, const std::string& details) {
    std::lock_guard<std::mutex> lock(mtx);
    alert_queue.push(type + "|" + details);
    cv.notify_one();
}

void AlertManager::alertLoop() {
    while (running) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this] { return !alert_queue.empty() || !running; });

        while (!alert_queue.empty()) {
            std::string alert = alert_queue.front();
            alert_queue.pop();
            lock.unlock();

            size_t sep = alert.find('|');
            std::string type = alert.substr(0, sep);
            std::string details = alert.substr(sep + 1);

            if (gpio) {
                gpio->pulseBuzzer(500);
            }

            if (type == "MOTION") {
                sendTelegram("Motion detected: " + details);
            }

            lock.lock();
        }
    }
}

void AlertManager::sendTelegram(const std::string& msg) {
    logger->info("Telegram alert: " + msg);
}

void AlertManager::sendEmail(const std::string& msg) {
    logger->info("Email alert: " + msg);
}
