#pragma once

#include <cstdint>

namespace robot::movers {
    bool begin();
    void drive(int8_t vx, int8_t vy, int8_t va);
}