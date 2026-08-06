// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "esp_err.h"

namespace motor {

enum class Direction {
    Clockwise,
    CounterClockwise,
};

enum class MotorStatus {
    Disabled,

    Starting,
    Stopping,

    Running,
    Ramping,

    Fault 
};

struct MotorState {
    MotorStatus status{MotorStatus::Disabled};

    bool motor_enabled{false};
    bool pfm_enabled{false};

    std::uint16_t current_freq_hz{0};
    std::uint16_t target_freq_hz{0};

    esp_err_t last_error;
};

} // namespace motor
