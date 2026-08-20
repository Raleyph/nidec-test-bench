// © 2026 Raleyph

#include <algorithm>
#include <cmath>

#include "esp_log.h"

#include "motor/config.hpp"
#include "motor/service.hpp"

namespace {

constexpr char TAG[] = "motor_service";

}

namespace motor
{

using namespace config;

//////////////////////////////////////////////////////////////////////////
// Public API

esp_err_t MotorService::start_task() {
    if (!initialized_ || task_handle_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    TaskHandle_t handle = nullptr;

    const BaseType_t result = xTaskCreate(
        &MotorService::task_entry,
        "motor_service",
        4096,
        this,
        5,
        &handle
    );

    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    task_handle_ = handle;

    return ESP_OK;
}

esp_err_t MotorService::initialize() {
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    // creating the command queue

    command_queue_ = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(Command));

    if (command_queue_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // initializing the driver

    esp_err_t err = driver_.initialize();

    if (err != ESP_OK) {
        return err;
    }

    state_ = {};
    state_.target_freq_hz = MIN_FREQ_HZ;
    state_.status = MotorStatus::Disabled;

    initialized_ = true;

    return ESP_OK;
}

esp_err_t MotorService::start()
{
    if (!ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    return enqueue(Command{
        .type = CommandType::Start,
    });
}

esp_err_t MotorService::stop() {
    if (!ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    return enqueue(Command{
        .type = CommandType::Stop,
    });
}

esp_err_t MotorService::set_speed(std::uint16_t rpm) {
    if (!ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (rpm < MIN_RPM || rpm > MAX_RPM) {
        return ESP_ERR_INVALID_ARG;
    }

    return enqueue(Command{
        .type = CommandType::SetSpeed,
        .rpm = rpm,
    });
}

MotorState MotorService::state() const {
    return state_;
}

//////////////////////////////////////////////////////////////////////////
// Entrypoint

void MotorService::task_entry(void * arg) {
    auto* service = static_cast<MotorService*>(arg);
    service->run();
}

void MotorService::run() {
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        pull_commands();

        const esp_err_t err = process_state();

        if (err != ESP_OK) {
            enter_fault(err);
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(RAMP_DELAY_MS)
        );
    }
}

void MotorService::enter_fault(const esp_err_t error)
{
    ESP_LOGE(
        TAG,
        "Entering FAULT: %s (0x%x)",
        esp_err_to_name(error),
        error
    );

    const esp_err_t pfm_err = driver_.set_pfm_enabled(false);
    const esp_err_t motor_err = driver_.set_enabled(false);

    if (pfm_err == ESP_OK) {
        state_.pfm_enabled = false;
    } else {
        ESP_LOGE(
            TAG,
            "Failed to disable PFM during fault handling: %s",
            esp_err_to_name(pfm_err)
        );
    }

    if (motor_err == ESP_OK) {
        state_.motor_enabled = false;
    } else {
        ESP_LOGE(
            TAG,
            "Failed to disable motor during fault handling: %s",
            esp_err_to_name(motor_err)
        );
    }

    state_.last_error = error;
    state_.status = MotorStatus::Fault;
}

esp_err_t MotorService::enqueue(const Command& command) {
    return xQueueSend(command_queue_, &command, 0) == pdTRUE
        ? ESP_OK
        : ESP_ERR_TIMEOUT;
}

void MotorService::pull_commands() {
    Command command{};

    while (xQueueReceive(command_queue_, &command, 0) == pdTRUE) {
        const esp_err_t err = handle_command(command);

        if (err != ESP_OK) {
            ESP_LOGW(
                TAG,
                "Command rejected: %s",
                esp_err_to_name(err)
            );
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// Command Handling

esp_err_t MotorService::handle_command(const Command& command) {
    switch (command.type) {
        case CommandType::Start:
            return handle_start_command();
        
        case CommandType::Stop:
            return handle_stop_command();
        
        case CommandType::SetSpeed:
            return handle_set_speed_command(command.rpm);
    }

    return ESP_ERR_INVALID_ARG;
}

esp_err_t MotorService::handle_start_command() {
    switch (state_.status) {
        case MotorStatus::Disabled:
            state_.status = MotorStatus::Starting;
            return ESP_OK;

        case MotorStatus::Starting:
        case MotorStatus::Running:
        case MotorStatus::Ramping:
            return ESP_OK;

        case MotorStatus::Stopping:
        case MotorStatus::Fault:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t MotorService::handle_stop_command() {
    switch (state_.status) {
        case MotorStatus::Disabled:
        case MotorStatus::Stopping:
            return ESP_OK;

        case MotorStatus::Fault:
            return ESP_ERR_INVALID_STATE;

        case MotorStatus::Starting:
        case MotorStatus::Running:
        case MotorStatus::Ramping:
            state_.target_freq_hz = MIN_FREQ_HZ;
            state_.status = MotorStatus::Stopping;
            return ESP_OK;
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t MotorService::handle_set_speed_command(const std::uint16_t rpm) {
    if (rpm < MIN_RPM || rpm > MAX_RPM) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_.status == MotorStatus::Stopping) {
        return ESP_OK;
    }

    if (state_.status == MotorStatus::Fault) {
        return ESP_ERR_INVALID_STATE;
    }

    const std::uint32_t target_freq_hz = rpm_to_frequency(rpm);

    if (target_freq_hz < MIN_FREQ_HZ || target_freq_hz > MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    state_.target_freq_hz = target_freq_hz;

    if (state_.status == MotorStatus::Running) {
        state_.status = MotorStatus::Ramping;
    }

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////
// Command Processing

esp_err_t MotorService::process_state() {
    switch(state_.status) {
        case MotorStatus::Disabled:
        case MotorStatus::Fault:
            return ESP_OK;
        
        case MotorStatus::Starting:
            return process_start();
        
        case MotorStatus::Stopping:
            return process_stop();
        
        case MotorStatus::Running:
        case MotorStatus::Ramping:
            return process_running();
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t MotorService::process_start() {

    // setting start frequency

    esp_err_t err = driver_.set_frequency(START_FREQ_HZ);
    if (err != ESP_OK) {
        return err;
    }

    state_.current_freq_hz = START_FREQ_HZ;

    // enabling PFM

    err = driver_.set_pfm_enabled(true);
    if (err != ESP_OK) {
        return err;
    }

    state_.pfm_enabled = true;

    // enabling the motor

    err = driver_.set_enabled(true);
    if (err != ESP_OK) {
        return err;
    }

    state_.motor_enabled = true;

    vTaskDelay(pdMS_TO_TICKS(1000));

    // setting state

    state_.status =
        state_.current_freq_hz != state_.target_freq_hz
            ? MotorStatus::Ramping
            : MotorStatus::Running;
    
    return ESP_OK;
}

esp_err_t MotorService::process_stop() {
    if (!state_.motor_enabled && !state_.pfm_enabled) {
        state_.current_freq_hz = 0;
        state_.target_freq_hz = MIN_FREQ_HZ;
        state_.status = MotorStatus::Disabled;
        return ESP_OK;
    }

    if (state_.current_freq_hz != state_.target_freq_hz) {
        return update_ramp();
    }

    // disabling PFM

    esp_err_t err = driver_.set_pfm_enabled(false);

    if (err != ESP_OK) {
        return err;
    }

    state_.pfm_enabled = false;

    // disabling the motor

    err = driver_.set_enabled(false);

    if (err != ESP_OK) {
        return err;
    }

    state_.motor_enabled = false;

    // setting state

    state_.current_freq_hz = 0;
    state_.target_freq_hz = MIN_FREQ_HZ;
    state_.status = MotorStatus::Disabled;

    return ESP_OK;
}

esp_err_t MotorService::process_running() {
    const esp_err_t err = update_ramp();

    if (err != ESP_OK) {
        return err;
    }

    state_.status =
        state_.current_freq_hz == state_.target_freq_hz
            ? MotorStatus::Running
            : MotorStatus::Ramping;

    return ESP_OK;
}

esp_err_t MotorService::update_ramp()
{
    const std::uint32_t current = state_.current_freq_hz;
    const std::uint32_t target = state_.target_freq_hz;

    if (current == target) {
        return ESP_OK;
    }

    // calculating next frequency

    std::uint32_t next_freq_hz{};

    if (current < target) {
        next_freq_hz = current + std::min(target - current, RAMP_STEP_HZ);
    } else {
        next_freq_hz = current - std::min(current - target, RAMP_STEP_HZ);
    }

    // setting next frequency

    const esp_err_t err = driver_.set_frequency(next_freq_hz);

    if (err != ESP_OK) {
        return err;
    }

    state_.current_freq_hz = next_freq_hz;

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////
// Helpers

bool MotorService::ready() const noexcept {
    return initialized_ && task_handle_ != nullptr && command_queue_ != nullptr;
}

std::uint32_t MotorService::rpm_to_frequency(std::uint16_t rpm) noexcept {
    return static_cast<std::uint32_t>(
        std::lround(static_cast<float>(rpm) * FREQ_PER_RPM)
    );
}

} // namespace motor
