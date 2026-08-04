// © 2026 Raleyph

#pragma once

#include <cstdint>

#include "esp_err.h"

#include "motor/types.hpp"

namespace motor {

//////////////////////////////////////////////////////////////////////////
// Common Motor Interface

class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    virtual esp_err_t initialize() = 0;
    virtual esp_err_t set_enabled(bool enabled) = 0;
    virtual esp_err_t set_pfm_enabled(bool enabled) = 0;
    virtual esp_err_t set_direction(Direction direction) = 0;
    virtual esp_err_t set_frequency(std::uint16_t freq_hz) = 0;
};

//////////////////////////////////////////////////////////////////////////
// Hardware Motor Implementation

class HardwareDriver final : public IMotorDriver {
public:
    esp_err_t initialize() override;
    esp_err_t set_enabled(bool enabled) override;
    esp_err_t set_pfm_enabled(bool enabled);
    esp_err_t set_direction(Direction direction) override;
    esp_err_t set_frequency(std::uint16_t freq_hz);
};

//////////////////////////////////////////////////////////////////////////
// Mock Motor Implementation

class MockDriver final : public IMotorDriver {
public:
    esp_err_t initialize() override;
    esp_err_t set_enabled(bool enabled) override;
    esp_err_t set_pfm_enabled(bool enabled);
    esp_err_t set_direction(Direction direction) override;
    esp_err_t set_frequency(std::uint16_t freq_hz) override;
};

} // namespace motor
