#pragma once
#include <Vec3.h>
#include <cstdint>
#include <commands.h>


namespace robot::movers {
    bool begin();
    void drive(MotionCommand& cmd);
    void goToTarget(const MotionCommand& cmd);
    Vec3 getCurrentVelocity();
}