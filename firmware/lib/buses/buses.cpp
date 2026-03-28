#include "buses.h"
#include <config.h>
#include <logging.h>

namespace robot::buses {
    TwoWire i2c_actuation(static_cast<uint8_t>(robot::config::I2CBusId::ACTUATION));
    TwoWire i2c_sensors(static_cast<uint8_t>(robot::config::I2CBusId::SENSORS));

    bool begin() {
        robot::logging::infof("buses", "init actuation bus id=%u sda=%u scl=%u",
                              static_cast<unsigned int>(robot::config::I2CBusId::ACTUATION),
                              robot::config::i2c_actuation_config.sdaPin,
                              robot::config::i2c_actuation_config.sclPin);
        if (!i2c_actuation.begin(robot::config::i2c_actuation_config.sdaPin, robot::config::i2c_actuation_config.sclPin)) {
            robot::logging::warn("buses", "actuation i2c begin failed");
            return false;
        }

        robot::logging::infof("buses", "init sensors bus id=%u sda=%u scl=%u",
                              static_cast<unsigned int>(robot::config::I2CBusId::SENSORS),
                              robot::config::i2c_sensors_config.sdaPin,
                              robot::config::i2c_sensors_config.sclPin);
        if (!i2c_sensors.begin(robot::config::i2c_sensors_config.sdaPin, robot::config::i2c_sensors_config.sclPin)) {
            robot::logging::warn("buses", "sensors i2c begin failed");
            return false;
        }

        robot::logging::info("buses", "i2c buses ready");
        return true;
    }

    TwoWire* get(robot::config::I2CBusId busId) {
        switch (busId) {
            case robot::config::I2CBusId::ACTUATION:
                return &i2c_actuation;
            case robot::config::I2CBusId::SENSORS:
                return &i2c_sensors;
            default:
                return &i2c_actuation;
        }
    }
}