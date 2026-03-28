#pragma once

#include <cstdint>

namespace robot::ioexpander {
    enum class CommandType : uint8_t {
        SetPin = 0,
        WriteMasked = 1,
    };

    struct Command {
        CommandType type = CommandType::SetPin;
        uint8_t pin = 0;
        bool level = false;
        uint16_t mask = 0;
        uint16_t value = 0;
    };

    bool begin();
    bool apply(const Command& command);
    uint16_t outputShadow();
}
