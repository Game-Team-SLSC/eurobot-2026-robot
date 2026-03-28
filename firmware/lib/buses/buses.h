#pragma once

#include <config.h>
#include <Wire.h>

namespace robot::buses {
    extern TwoWire i2c_actuation;
    extern TwoWire i2c_sensors;

    bool begin();
    TwoWire* get(robot::config::I2CBusId busId);
}