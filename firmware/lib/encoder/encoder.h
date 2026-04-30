#pragma once

#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>

namespace robot::encoder {
    enum class Position: uint8_t {
        A_LOW_B_LOW = 0b00,
        A_HIGH_B_LOW = 0b10,
        A_HIGH_B_HIGH = 0b11,
        A_LOW_B_HIGH = 0b01
    };

    void begin();
    Position read();
}