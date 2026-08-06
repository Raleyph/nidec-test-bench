// © Raleyph 2026

#pragma once

#include <cstdint>

namespace motor
{

enum class CommandType : std::uint8_t {
    Start,
    Stop,
    SetSpeed
};

struct Command {
    CommandType type;
    std::uint16_t rpm{};
};

} // namespace motor
