#pragma once

#include <ADS1X15.h>
#include <cstdint>

namespace robot::battery {
    ADS1015 adc;

    float getVoltage();
    uint8_t getPercentage();
}