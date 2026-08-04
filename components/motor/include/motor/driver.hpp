// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "esp_err.h"

namespace motor {

enum class Direction {
    Clockwise,
    CounterClockwise,
};

struct State {
    bool enabled;
    std::uint32_t current_freq_hz;
    Direction direction;
};

class Driver {
public:
    virtual ~Driver() = default;

    virtual esp_err_t initialize() = 0;
    virtual esp_err_t set_enabled(bool enabled) = 0;
    virtual esp_err_t set_speed(std::uint16_t rpm) = 0;
    virtual esp_err_t set_direction(Direction direction) = 0;
    virtual esp_err_t stop() = 0;

    [[nodiscard]]
    const State& state() const {
        return state_;
    }

protected:
    State state_{};
};

class HardwareDriver final : public Driver {
public:
    esp_err_t initialize() override;
    esp_err_t set_enabled(bool enabled) override;
    esp_err_t set_speed(std::uint16_t rpm) override;
    esp_err_t set_direction(Direction direction) override;
    esp_err_t stop() override;

private:
    esp_err_t set_frequency(std::uint16_t freq_hz);
    esp_err_t ramp_to_frequency(std::uint16_t target_freq, uint32_t ramp_time_ms);
    esp_err_t ramp_stop(std::uint32_t ramp_time_ms);

    std::uint32_t calculate_ramp_time(std::uint16_t start_freq_hz, std::uint16_t target_freq_hz);
    std::uint32_t calculate_step_count(std::uint16_t start_freq_hz, std::uint16_t target_freq_hz);
    TickType_t calculate_step_delay(std::uint32_t ramp_time_ms, std::uint32_t step_count);

    esp_err_t set_signal_enabled(bool enabled);
};

class MockDriver final : public Driver {
public:
    esp_err_t initialize() override;
    esp_err_t set_enabled(bool enabled) override;
    esp_err_t set_speed(std::uint16_t rpm) override;
    esp_err_t set_direction(Direction direction) override;
    esp_err_t stop() override;
};

}