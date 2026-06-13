#include "surveillance.hpp"

GPIOManager::GPIOManager(SystemStatus* s, Logger* l) : status(s), logger(l) {}

GPIOManager::~GPIOManager() {
    stop();
}

bool GPIOManager::init() {
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        logger->error("Failed to open GPIO chip");
        return false;
    }

    pir_line = gpiod_chip_get_line(chip, config::PIR_PIN);
    ir_line = gpiod_chip_get_line(chip, config::IR_LED_PIN);
    buzzer_line = gpiod_chip_get_line(chip, config::BUZZER_PIN);
    status_line = gpiod_chip_get_line(chip, config::STATUS_LED_PIN);

    if (pir_line) gpiod_line_request_input(pir_line, "surveillance");
    if (ir_line) gpiod_line_request_output(ir_line, "surveillance", 0);
    if (buzzer_line) gpiod_line_request_output(buzzer_line, "surveillance", 0);
    if (status_line) gpiod_line_request_output(status_line, "surveillance", 0);

    logger->info("GPIO initialized");
    return true;
}

void GPIOManager::start() {
    running = true;
    monitor_thread = std::thread(&GPIOManager::monitorLoop, this);
}

void GPIOManager::stop() {
    running = false;
    if (monitor_thread.joinable()) monitor_thread.join();

    if (ir_line) gpiod_line_set_value(ir_line, 0);
    if (buzzer_line) gpiod_line_set_value(buzzer_line, 0);
    if (status_line) gpiod_line_set_value(status_line, 0);

    if (pir_line) gpiod_line_release(pir_line);
    if (ir_line) gpiod_line_release(ir_line);
    if (buzzer_line) gpiod_line_release(buzzer_line);
    if (status_line) gpiod_line_release(status_line);
    if (chip) gpiod_chip_close(chip);
}

void GPIOManager::monitorLoop() {
    while (running) {
        if (pir_line) {
            int val = gpiod_line_get_value(pir_line);
            if (val == 1 && !status->pir_triggered.load()) {
                status->pir_triggered = true;
                logger->info("PIR motion detected!");
                pulseBuzzer(200);
                status->motion_events++;
            } else if (val == 0) {
                status->pir_triggered = false;
            }
        }

        if (status_line) {
            bool blink = status->is_recording.load() || status->motion_detected.load();
            gpiod_line_set_value(status_line, blink ? 1 : 0);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void GPIOManager::setIR(bool state) {
    if (ir_line) {
        gpiod_line_set_value(ir_line, state ? 1 : 0);
        status->night_mode = state;
        logger->info(std::string("IR LEDs: ") + (state ? "ON" : "OFF"));
    }
}

void GPIOManager::setBuzzer(bool state) {
    if (buzzer_line) {
        gpiod_line_set_value(buzzer_line, state ? 1 : 0);
    }
}

void GPIOManager::setStatusLED(bool state) {
    if (status_line) {
        gpiod_line_set_value(status_line, state ? 1 : 0);
    }
}

void GPIOManager::pulseBuzzer(int ms) {
    setBuzzer(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    setBuzzer(false);
}

bool GPIOManager::readPIR() {
    if (pir_line) {
        return gpiod_line_get_value(pir_line) == 1;
    }
    return false;
}
