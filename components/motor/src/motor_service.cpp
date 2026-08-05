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

    // initializing the driver

    esp_err_t err = driver_.initialize();

    if (err != ESP_OK) {
        return err;
    }

    // setting operational direction

    err = driver_.set_direction(Direction::Clockwise);

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
    if (!initialized_ || task_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

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

esp_err_t MotorService::stop() {
    if (!initialized_ || task_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    switch (state_.status) {
        case MotorStatus::Disabled:
            return ESP_OK;

        case MotorStatus::Fault:
            return ESP_ERR_INVALID_STATE;

        case MotorStatus::Stopping:
            return ESP_OK;

        default:
            state_.target_freq_hz = MIN_FREQ_HZ;
            state_.status = MotorStatus::Stopping;
            return ESP_OK;
    }
}

esp_err_t MotorService::set_speed(std::uint16_t rpm) {
    if (!initialized_ || task_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (
        state_.status == MotorStatus::Stopping ||
        state_.status == MotorStatus::Fault
    ) {
        return ESP_ERR_INVALID_STATE;
    }

    // validating RPM

    if (rpm < MIN_RPM || rpm > MAX_RPM) {
        return ESP_ERR_INVALID_ARG;
    }

    // calculating and validating frequency

    const std::uint32_t target_freq_hz = rpm_to_frequency(rpm);

    if (
        target_freq_hz < MIN_FREQ_HZ ||
        target_freq_hz > MAX_FREQ_HZ
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    // setting target frequency and state

    state_.target_freq_hz = target_freq_hz;

    if (state_.status == MotorStatus::Running) {
        state_.status = MotorStatus::Ramping;
    }

    return ESP_OK;
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
        const esp_err_t err = process();

        if (err != ESP_OK) {
            enter_fault(err);
        }

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(RAMP_DELAY_MS)
        );
    }
}

//////////////////////////////////////////////////////////////////////////
// Ramp Implementation

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
// Command Processing

esp_err_t MotorService::process() {
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

    // setting min frequency

    esp_err_t err = driver_.set_frequency(MIN_FREQ_HZ);

    if (err != ESP_OK) {
        return err;
    }

    state_.current_freq_hz = MIN_FREQ_HZ;

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

    // setting state

    state_.status =
        state_.target_freq_hz > MIN_FREQ_HZ
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

//////////////////////////////////////////////////////////////////////////
// Faults Handling

void MotorService::enter_fault(const esp_err_t error)
{
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

//////////////////////////////////////////////////////////////////////////
// Calculations

std::uint32_t MotorService::rpm_to_frequency(std::uint16_t rpm) noexcept {
    return static_cast<std::uint32_t>(
        std::lround(static_cast<float>(rpm) * RPM_TO_FREQ_RATE)
    );
}

} // namespace motor
