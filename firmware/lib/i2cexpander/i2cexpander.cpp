#include "i2cexpander.h"

#include <Arduino.h>
#include <TCA9548.h>

#include <buses.h>
#include <config.h>

namespace {
    TCA9548 logicMux(robot::config::tca9548_logic_i2c_config.address, robot::buses::get(robot::config::tca9548_logic_i2c_config.busId));
}

TCA9548* getCtrl(robot::config::I2CController ctrl) {
    switch (ctrl) {
        case robot::config::I2CController::TCA9548_LOGIC:
            return &logicMux;
        default:
            return nullptr;
    }
}

bool beginTarget(TCA9548& target) {
    const bool beginOk = target.begin();

    return beginOk;
}

namespace robot::i2cexpander {
bool begin() {
    bool ok = true;
    bool hasConfiguredTarget = false;
    
    if (!beginTarget(logicMux)) {
        ok = false;
    }

    if (!ok) {
        Serial.println("[i2cexpander] init failed");
    }

    return ok;
}

bool apply(const I2CExpanderCommand& command) {
    TCA9548* targetMux = getCtrl(command.controller);
    if (targetMux == nullptr) {
        return false;
    }

    // Select channel exclusively: disable all first
    for (uint8_t ch = 0; ch < 8; ++ch) {
        targetMux->disableChannel(ch);
    }

    return targetMux->enableChannel(command.channel);
}



} // namespace robot::i2cexpander
