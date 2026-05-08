#pragma once

#include <commands.h>
#include <cstdint>
#include <commands.h>

namespace robot::pwmcontroller {
    constexpr uint8_t MAX_CMD = 8;

    bool begin();
    bool apply(const PWMCommand& command);
}