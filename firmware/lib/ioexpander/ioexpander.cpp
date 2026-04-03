#include "ioexpander.h"

#include <TCA9555.h>

#include <buses.h>
#include <config.h>

namespace {
TCA9555 logic_mux(robot::config::tca9555_logic_i2c_config.address, robot::buses::get(robot::config::tca9555_logic_i2c_config.busId));
TCA9555 kinetic_mux(robot::config::tca9555_kinetic_i2c_config.address, robot::buses::get(robot::config::tca9555_kinetic_i2c_config.busId));
} // namespace

namespace robot::ioexpander {
    bool begin() {
        Serial.println(logic_mux.begin()? "[ioexpander] initialized" : "[ioexpander] initialization failed");
        Serial.println(kinetic_mux.begin()? "[ioexpander] initialized" : "[ioexpander] initialization failed");
        return true;
    }

    bool apply(const IOExpanderCommand& command) {
        TCA9555* mux;
        switch (command.expander) {
            case robot::config::IOExpander::LOGIC:
                mux = &logic_mux;
                break;
            case robot::config::IOExpander::KINETIC:
                mux = &kinetic_mux;
                break;
        }
        mux->pinMode1(command.pin, OUTPUT);
        
        return (mux->write1(command.pin, command.level ? HIGH : LOW));
    }
}
