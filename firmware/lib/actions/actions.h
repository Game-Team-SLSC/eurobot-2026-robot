#pragma once

#include <cstdint>

enum class Action: uint8_t {
    IDLE,
    TURN_FRONT,
    STOCK_FRONT,
    RELEASE_FRONT,
    TURN_TWO_FRONT,

    TURN_BACK,
    STOCK_BACK,
    RELEASE_BACK,
    TURN_TWO_BACK,

    _ACTION_COUNT
};

namespace robot::actions {
    void turn_front();
    void stock_front();
    void release_front();
    void turn_two_front();

    void turn_back();
    void stock_back();
    void release_back();
    void turn_two_back();
}