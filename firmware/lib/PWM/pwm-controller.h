#pragma once

#include <config.h>

#include <cstdint>

namespace robot::pwmcontroller {
    struct Command {
        robot::config::PWMController controller;
        uint8_t pin;
        uint16_t value = 0;
    };
    
    bool begin();
    bool apply(const Command& command);
}
