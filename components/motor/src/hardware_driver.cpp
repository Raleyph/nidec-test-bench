// © 2026 Raleyph

#include <cmath>

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"

#include "motor/driver.hpp"

namespace
{
    constexpr gpio_num_t ENABLE_GPIO = GPIO_NUM_3;
    constexpr gpio_num_t DIRECTION_GPIO = GPIO_NUM_4;
    constexpr gpio_num_t PFM_GPIO = GPIO_NUM_2;

    constexpr std::uint16_t MIN_FREQ_HZ = 1000;
    constexpr std::uint16_t MAX_FREQ_HZ = 16000;

    constexpr std::uint8_t MIN_RPM = 150;
    constexpr float RPM_RATE = MIN_FREQ_HZ / MIN_RPM;

    constexpr std::uint8_t RAMP_STEP_HZ = 100;

    constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
    constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
    constexpr ledc_timer_bit_t LEDC_TIMER_BIT = LEDC_TIMER_10_BIT;
    constexpr ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;

    constexpr std::uint32_t DUTY_50_PERCENT = 512;
} // namespace


namespace motor {

//////////////////////////////////////////////////////////////////////////
// Public API

esp_err_t HardwareDriver::initialize() {
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = MIN_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&timer_config);

    if (err != ESP_OK) {
        return err;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = PFM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,

        .duty = 0,
        .hpoint = 0
    };

    err = ledc_channel_config(&channel_config);

    if (err != ESP_OK) {
        return err;
    }

    state_.enabled = false;
    state_.current_freq_hz = MIN_FREQ_HZ;

    return ESP_OK;
}

esp_err_t HardwareDriver::set_enabled(bool enabled) {
    const esp_err_t err = gpio_set_level(ENABLE_GPIO, enabled ? 1 : 0);

    if (err != ESP_OK) {
        return err;
    }

    state_.enabled = enabled;

    return ESP_OK;
}

esp_err_t HardwareDriver::set_speed(std::uint16_t rpm) {
    const std::uint16_t target_freq = static_cast<std::uint16_t>(std::lround(rpm * RPM_RATE));
    const std::uint32_t ramp_time_ms = calculate_ramp_time(state_.current_freq_hz, target_freq);
    return ramp_to_frequency(target_freq, ramp_time_ms);
}

esp_err_t HardwareDriver::set_direction(Direction direction) {
    const std::uint8_t level = direction == Direction::Clockwise ? 0 : 1;
    const esp_err_t err = gpio_set_level(DIRECTION_GPIO, level);

    if (err != ESP_OK) {
        return err;
    }

    state_.direction = direction;

    return ESP_OK;
}

esp_err_t HardwareDriver::stop() {
    const std::uint32_t ramp_time_ms = calculate_ramp_time(state_.current_freq_hz, MIN_FREQ_HZ);
    return ramp_stop(ramp_time_ms);
}

//////////////////////////////////////////////////////////////////////////
// Internal PFM Interaction

esp_err_t HardwareDriver::set_frequency(std::uint16_t freq_hz) {
    if (freq_hz < MIN_FREQ_HZ || freq_hz > MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t err = ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);

    if (err != ESP_OK) {
        return err;
    }

    state_.current_freq_hz = freq_hz;

    return ESP_OK;
}

esp_err_t HardwareDriver::ramp_to_frequency(uint16_t target_freq_hz, uint32_t ramp_time_ms) {
    if (target_freq_hz < MIN_FREQ_HZ || target_freq_hz > MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    if (ramp_time_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // check is motor running

    if (!state_.enabled) {
        esp_err_t err = set_frequency(MIN_FREQ_HZ);

        if (err != ESP_OK) {
            return err;
        }

        err = set_signal_enabled(true);

        if (err != ESP_OK) {
            return err;
        }

        state_.enabled = true;
    }

    // calculate intermediate values

    const std::uint16_t start_freq_hz = state_.current_freq_hz;
    const std::uint32_t step_count = calculate_step_count(start_freq_hz, target_freq_hz);

    if (step_count == 0) {
        return ESP_OK;
    }

    const TickType_t step_delay = calculate_step_delay(ramp_time_ms, step_count);

    std::uint16_t freq_hz = start_freq_hz;

    // smoothly set target frequency

    while (freq_hz != target_freq_hz) {
        if (freq_hz < target_freq_hz) {
            const std::uint16_t remaining_hz = target_freq_hz - freq_hz;

            freq_hz += remaining_hz > RAMP_STEP_HZ
                ? RAMP_STEP_HZ
                : remaining_hz;
        } else {
            const std::uint16_t remaining_hz = freq_hz - target_freq_hz;

            freq_hz -= remaining_hz > RAMP_STEP_HZ
                ? RAMP_STEP_HZ
                : remaining_hz;
        }

        const esp_err_t err = set_frequency(freq_hz);

        if (err != ESP_OK) {
            return err;
        }

        vTaskDelay(step_delay);
    }

    return ESP_OK;
}

esp_err_t HardwareDriver::ramp_stop(std::uint32_t ramp_time_ms) {
    if (ramp_time_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state_.enabled) {
        return ESP_OK;
    }

    esp_err_t err = ramp_to_frequency(MIN_FREQ_HZ, ramp_time_ms);

    if (err != ESP_OK) {
        return err;
    }

    err = set_signal_enabled(false);

    if (err != ESP_OK) {
        return err;
    }

    state_.enabled = false;
    state_.current_freq_hz = MIN_FREQ_HZ;

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////
// Internal Calculating

std::uint32_t HardwareDriver::calculate_ramp_time(std::uint16_t start_freq_hz, std::uint16_t target_freq_hz) {

}

std::uint32_t HardwareDriver::calculate_step_count(std::uint16_t start_freq_hz, std::uint16_t target_freq_hz) {
    const std::uint32_t difference_hz =
        start_freq_hz > target_freq_hz
            ? start_freq_hz - target_freq_hz
            : target_freq_hz - start_freq_hz;
    
    if (difference_hz == 0) {
        return 0;
    }

    return (difference_hz + RAMP_STEP_HZ - 1) / RAMP_STEP_HZ;
}

TickType_t HardwareDriver::calculate_step_delay(std::uint32_t ramp_time_ms, std::uint32_t step_count) {
    if (step_count == 0) {
        return 0;
    }

    const uint32_t delay_ms = ramp_time_ms / step_count;
    const TickType_t delay_ticks = pdMS_TO_TICKS(delay_ms);

    return delay_ticks > 0 ? delay_ticks : 1;
}

//////////////////////////////////////////////////////////////////////////
// Etc

esp_err_t HardwareDriver::set_signal_enabled(bool enabled) {
    const std::uint32_t duty = enabled ? DUTY_50_PERCENT : 0;

    esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);

    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

}
