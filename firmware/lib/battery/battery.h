#pragma once

#include <ADS1X15.h>
#include <cstdint>

namespace robot::battery {
    struct BatteryStatus {
    uint8_t percentage;
    uint16_t voltage_mv;
    
    uint8_t cell_1_percentage;
    uint8_t cell_2_percentage;
    uint8_t cell_3_percentage;
    uint8_t cell_4_percentage;
    
    uint16_t cell_1_voltage_mv;
    uint16_t cell_2_voltage_mv;
    uint16_t cell_3_voltage_mv;
    uint16_t cell_4_voltage_mv;
};

    bool begin();
    BatteryStatus getStatus();
}