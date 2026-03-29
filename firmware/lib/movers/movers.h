#pragma once

#include <cstdint>


namespace robot::movers {
    struct Vec3 {
        int16_t forward;
        int16_t strafe;
        int16_t rotate;
    };

    bool begin();
    void drive(int8_t vx, int8_t vy, int8_t va);
    Vec3 getCurrentVelocity();
}