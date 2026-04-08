#pragma once

#include <cstdint>

enum class Action: uint8_t {
    IDLE,
    TURN,
    STOCK,
    RELEASE,

    _ACTION_COUNT
};

namespace robot::actions {
    void turn();
    void stock();
    void release();
}