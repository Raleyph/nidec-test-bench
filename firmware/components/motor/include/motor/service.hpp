// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "motor/types.hpp"
#include "motor/driver.hpp"
#include "motor/commands.hpp"

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

    [[nodiscard]] MotorState state() const;

private:
    static void task_entry(void* arg);
    void run();
    void enter_fault(esp_err_t error);

    esp_err_t enqueue(const Command& command);

    void pull_commands();

    esp_err_t handle_command(const Command& command);
    esp_err_t handle_start_command();
    esp_err_t handle_stop_command();
    esp_err_t handle_set_speed_command(const std::uint16_t rpm);
    
    esp_err_t process_state();
    esp_err_t process_start();
    esp_err_t process_stop();
    esp_err_t process_running();
    esp_err_t update_ramp();

    [[nodiscard]] bool ready() const noexcept;
    [[nodiscard]] static std::uint32_t rpm_to_frequency(std::uint16_t rpm) noexcept;

private:
    IMotorDriver& driver_;
    MotorState state_{};

    TaskHandle_t task_handle_{nullptr};
    QueueHandle_t command_queue_{nullptr};

    bool initialized_{false};
};

} // namespace motor
