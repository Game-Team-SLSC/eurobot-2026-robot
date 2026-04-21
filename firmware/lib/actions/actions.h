#pragma once

#include <cstdint>

enum class Action: uint8_t {
    IDLE,
    TURN,
    STOCK,
    RELEASE,
    TURN_TWO,

    _ACTION_COUNT
};

namespace robot::actions {
    void turn();
    void stock();
    void release();
    void turn_two();
}