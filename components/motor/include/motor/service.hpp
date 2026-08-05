// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor/types.hpp"
#include "motor/driver.hpp"

namespace motor
{

class MotorService {
public:
    explicit MotorService(IMotorDriver& driver) : driver_(driver) {}

    esp_err_t start_task();
    esp_err_t initialize();

    esp_err_t start();
    esp_err_t stop();

    esp_err_t set_speed(std::uint16_t rpm);

private:
    static void task_entry(void* arg);

    void run();

    esp_err_t update_ramp();

    esp_err_t process();
    
    esp_err_t process_start();
    esp_err_t process_stop();

    esp_err_t process_running();

    void enter_fault(esp_err_t error);

    [[nodiscard]] static std::uint32_t rpm_to_frequency(std::uint16_t rpm) noexcept;

private:
    bool initialized_{false};

    IMotorDriver& driver_;
    MotorState state_{};

    TaskHandle_t task_handle_{nullptr};
};

} // namespace motor
