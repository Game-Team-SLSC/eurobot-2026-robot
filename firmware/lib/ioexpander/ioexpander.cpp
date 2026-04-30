#include "ioexpander.h"

#include <TCA9555.h>
#include <queues.h>
#include <buses.h>
#include <config.h>
#include <Logger.h>

namespace {
TCA9555 logic_mux(robot::config::tca9555_logic_i2c_config.address, robot::buses::get(robot::config::tca9555_logic_i2c_config.busId));
TCA9555 kinetic_mux(robot::config::tca9555_kinetic_i2c_config.address, robot::buses::get(robot::config::tca9555_kinetic_i2c_config.busId));
} // namespace

namespace robot::ioexpander {
    bool begin() {
        bool ok = logic_mux.begin() && kinetic_mux.begin();
        pinMode(47, INPUT);

        if (!ok) {
            error("ioexpander", "Initialization failed");
            return false;
        }

        info("ioexpander", "Initialized");
        
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

        if (command.action == IOAction::READ) {
            mux->pinMode1(command.pin, INPUT);
            int32_t value = mux->read1(command.pin);
            xQueueSend(robot::queues::io_response_queue, &value, 0);
            return true;
        } else {
            mux->pinMode1(command.pin, OUTPUT);
            return (mux->write1(command.pin, command.level ? HIGH : LOW));
        }
    }
}
