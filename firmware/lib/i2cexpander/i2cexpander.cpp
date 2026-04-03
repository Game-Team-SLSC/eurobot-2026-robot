#include "i2cexpander.h"

#include <Arduino.h>
#include <TCA9548.h>

#include <buses.h>
#include <config.h>

namespace {
    TCA9548 sensorsMux(robot::config::tca9548_sensors_i2c_config.address, robot::buses::get(robot::config::tca9548_sensors_i2c_config.busId));
    TCA9548 logicMux(robot::config::tca9548_logic_i2c_config.address, robot::buses::get(robot::config::tca9548_logic_i2c_config.busId));
}

TCA9548* getMuxForBus(robot::config::I2CBusId busId) {
    switch (busId) {
        case robot::config::I2CBusId::SENSORS:
            return &sensorsMux;
        case robot::config::I2CBusId::ACTUATION:
            return &logicMux;
        default:
            return nullptr;
    }
}

bool beginTarget(TCA9548& target) {
    const bool beginOk = target.begin();
    const bool connected = target.isConnected();
    
    if (beginOk && connected) {
        return false;
    }

    return true;
}

namespace robot::i2cexpander {
bool begin() {
    bool ok = true;
    bool hasConfiguredTarget = false;

    if (!beginTarget(sensorsMux)) {
        ok = false;
    }
    if (!beginTarget(logicMux)) {
        ok = false;
    }

    if (!ok) {
        Serial.println("[i2cexpander] init failed");
    }

    return ok;
}

bool apply(const I2CExpanderCommand& command) {
    TCA9548* targetMux = getMuxForBus(command.controller.busId);
    if (targetMux == nullptr) {
        return false;
    }

    targetMux->enableChannel(command.channel);
    // if (command.callback != nullptr) {
    //     command.callback();
    // }
    targetMux->disableChannel(command.channel);

    return true;
}

bool apply(const CommandBatch<I2CExpanderCommand>& batch) {
    bool success = true;
    for (uint8_t i = 0; i < batch.count; ++i) {
        if (!apply(batch.commands[i])) {
            success = false;
        }
    }
    return success;
}



} // namespace robot::i2cexpander
