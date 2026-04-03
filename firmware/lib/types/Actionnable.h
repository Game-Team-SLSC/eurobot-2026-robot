#pragma once

#include <cstdint>

enum class Actionnable: uint8_t {
    BACK_LEFT_TURNER,
    FRONT_LEFT_TURNER,
    BACK_RIGHT_TURNER,
    FRONT_RIGHT_TURNER,
    BACK_LEFT_GRABBER,
    FRONT_LEFT_GRABBER,
    BACK_RIGHT_GRABBER,
    FRONT_RIGHT_GRABBER,
};