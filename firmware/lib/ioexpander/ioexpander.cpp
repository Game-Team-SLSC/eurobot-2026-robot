#include "ioexpander.h"

#include <TCA9555.h>

#include <buses.h>
#include <config.h>
#include <logging.h>

namespace {
TCA9555 g_tca(robot::config::tca9555_i2c_config.address,
               robot::buses::get(robot::config::tca9555_i2c_config.busId));

uint16_t g_outputShadow = 0;
bool g_ready = false;

bool writeShadow() {
    const bool ok = g_tca.write16(g_outputShadow);
    if (!ok) {
        robot::logging::warnf("ioexpander", "write16 failed err=0x%02X", static_cast<unsigned int>(g_tca.lastError()));
    }
    return ok;
}
} // namespace

namespace robot::ioexpander {
    bool begin() {
        Serial.println(g_tca.begin()? "[ioexpander] initialized" : "[ioexpander] initialization failed");
        g_ready = true;

        return true;
    }

    bool apply(const Command& command) {
        if (!g_ready) {
            robot::logging::warn("ioexpander", "apply called before init");
            return false;
        }

        switch (command.type) {
            case CommandType::SetPin: {
                g_tca.pinMode1(command.pin, OUTPUT);
                Serial.printf("[ioexpander] SetPin pin=%u level=%u\n",
                              static_cast<unsigned int>(command.pin),
                              static_cast<unsigned int>(command.level));
                
                return (g_tca.write1(command.pin, command.level ? HIGH : LOW));
            }

            case CommandType::WriteMasked: {
                g_outputShadow = static_cast<uint16_t>((g_outputShadow & ~command.mask) |
                                                       (command.value & command.mask));
                robot::logging::infof("ioexpander", "write masked mask=0x%04X value=0x%04X shadow=0x%04X",
                                      command.mask,
                                      command.value,
                                      g_outputShadow);
                return writeShadow();
            }

            default:
                robot::logging::warn("ioexpander", "unknown command type");
                return false;
        }
    }

    uint16_t outputShadow() {
        return g_outputShadow;
    }
}
