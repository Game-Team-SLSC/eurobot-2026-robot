#include "buses.h"
#include <config.h>

namespace robot::buses {
    TwoWire i2c_actuation(static_cast<uint8_t>(robot::config::I2CBusId::ACTUATION));
    TwoWire i2c_sensors(static_cast<uint8_t>(robot::config::I2CBusId::SENSORS));

    bool begin() {
        if (!i2c_actuation.begin(robot::config::i2c_actuation_config.sdaPin, robot::config::i2c_actuation_config.sclPin)) {
            return false;
        }

        if (!i2c_sensors.begin(robot::config::i2c_sensors_config.sdaPin, robot::config::i2c_sensors_config.sclPin)) {
            return false;
        }

        return true;
    }

    TwoWire* get(robot::config::I2CBusId busId) {
        switch (busId) {
            case robot::config::I2CBusId::ACTUATION:
                return &i2c_actuation;
            case robot::config::I2CBusId::SENSORS:
                return &i2c_sensors;
            default:
                // Handle invalid busId if necessary
                return &i2c_actuation; // Default to actuation bus
        }
    }
}