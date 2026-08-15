// © 2026 Raleyph

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"

#include "motor/config.hpp"
#include "motor/driver.hpp"

namespace motor {

using namespace config;

esp_err_t HardwareDriver::initialize() {
    gpio_config_t io_config{};

    io_config.pin_bit_mask = (1ULL << ENABLE_GPIO) | (1ULL << DIRECTION_GPIO);
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pull_up_en = GPIO_PULLUP_DISABLE;
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_config);

    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(gpio_set_level(ENABLE_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(DIRECTION_GPIO, 0));

    const ledc_timer_config_t timer_config{
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_BIT,
        .timer_num = LEDC_TIMER,
        .freq_hz = MIN_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    err = ledc_timer_config(&timer_config);

    if (err != ESP_OK) {
        return err;
    }

    const ledc_channel_config_t channel_config{
        .gpio_num = PFM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {},
        .deconfigure = false,
    };

    err = ledc_channel_config(&channel_config);

    if (err != ESP_OK) {
        return err;
    }

    applied_freq_hz_ = MIN_FREQ_HZ;

    return ESP_OK;
}

esp_err_t HardwareDriver::set_enabled(bool enabled) {
    const esp_err_t err = gpio_set_level(ENABLE_GPIO, enabled ? 1 : 0);

    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t HardwareDriver::set_pfm_enabled(bool enabled) {
    const std::uint32_t duty = enabled ? DUTY_50_PERCENT : 0;

    esp_err_t err = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);

    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

esp_err_t HardwareDriver::set_direction(Direction direction) {
    const std::uint8_t level = direction == Direction::Clockwise ? 0 : 1;
    const esp_err_t err = gpio_set_level(DIRECTION_GPIO, level);

    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t HardwareDriver::set_frequency(std::uint16_t freq_hz) {
    if (freq_hz < MIN_FREQ_HZ || freq_hz > MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    if (freq_hz > applied_freq_hz_ + MAX_FREQ_STEP_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t err = ledc_set_freq(LEDC_MODE, LEDC_TIMER, freq_hz);

    if (err != ESP_OK) {
        return err;
    }

    applied_freq_hz_ = freq_hz;

    return ESP_OK;
}

} // namespace motor
