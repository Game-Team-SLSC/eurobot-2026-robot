#include "buses.h"
#include <Arduino.h>
#include <config.h>
#include <Logger.h>
#include "spi_mutex.h"

namespace robot::buses {
    TwoWire i2c_actuation(static_cast<uint8_t>(robot::config::I2CBusId::ACTUATION));
    TwoWire i2c_sensors(static_cast<uint8_t>(robot::config::I2CBusId::SENSORS));

    bool begin() {
        robot::spi_mutex::begin();

        SPI.begin(robot::config::spi_sck_pin, robot::config::spi_miso_pin, robot::config::spi_mosi_pin);

        bool ok = i2c_actuation.begin(robot::config::i2c_actuation_config.sda_pin, robot::config::i2c_actuation_config.scl_pin) &&
        i2c_sensors.begin(robot::config::i2c_sensors_config.sda_pin, robot::config::i2c_sensors_config.scl_pin);

        if (!ok) {
            error("buses", "Initialization failed");
            return false;
        }

        info("buses", "Initialized");
        return true;
    }

    TwoWire* get(robot::config::I2CBusId busId) {
        switch (busId) {
            case robot::config::I2CBusId::ACTUATION:
                return &i2c_actuation;
            case robot::config::I2CBusId::SENSORS:
                    return &i2c_sensors;
            default:
                return nullptr;
        }
    }
}