// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/ledc.h"

namespace motor::config
{

// GPIO

constexpr gpio_num_t ENABLE_GPIO = GPIO_NUM_1;
constexpr gpio_num_t PFM_GPIO = GPIO_NUM_2;

// Motor

constexpr std::uint16_t START_FREQ_HZ = 18'000;

constexpr std::uint16_t MIN_RPM = 500;
constexpr std::uint16_t MIN_FREQ_HZ = 3'333;

constexpr std::uint16_t MAX_RPM = 3'000;
constexpr std::uint16_t MAX_FREQ_HZ = 20'000;

constexpr float FREQ_PER_RPM = static_cast<float>(MAX_FREQ_HZ) / static_cast<float>(MAX_RPM);
constexpr std::uint16_t MAX_FREQ_STEP_HZ = 100;

constexpr std::uint32_t RAMP_STEP_HZ = 50;
constexpr std::uint32_t RAMP_DELAY_MS = 20;

// Command Queue

constexpr std::size_t COMMAND_QUEUE_LENGTH = 8;

// LEDC

constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
constexpr ledc_timer_bit_t LEDC_TIMER_BIT = LEDC_TIMER_10_BIT;
constexpr ledc_channel_t LEDC_CHANNEL = LEDC_CHANNEL_0;

constexpr std::uint32_t DUTY_50_PERCENT = 512;

} // namespace motor::config
