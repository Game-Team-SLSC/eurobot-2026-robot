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
        robot::logging::infof("ioexpander", "init addr=0x%02X", robot::config::tca9555_i2c_config.address);

        if (!g_tca.begin()) {
            robot::logging::warn("ioexpander", "begin failed");
            return false;
        }

        if (!g_tca.isConnected()) {
            robot::logging::warn("ioexpander", "device not connected");
            return false;
        }

        for (uint8_t pin = 0; pin < 16; ++pin) {
            if (!g_tca.pinMode1(pin, OUTPUT)) {
                robot::logging::warnf("ioexpander", "pinMode failed pin=%u", pin);
                return false;
            }
            if (!g_tca.write1(pin, LOW)) {
                robot::logging::warnf("ioexpander", "initial write failed pin=%u", pin);
                return false;
            }
        }

        g_outputShadow = 0;
        g_ready = true;
        robot::logging::info("ioexpander", "ready");
        return true;
    }

    bool apply(const Command& command) {
        if (!g_ready) {
            robot::logging::warn("ioexpander", "apply called before init");
            return false;
        }

        switch (command.type) {
            case CommandType::SetPin: {
                if (command.pin >= 16) {
                    robot::logging::warnf("ioexpander", "invalid pin=%u", command.pin);
                    return false;
                }

                const uint16_t bit = static_cast<uint16_t>(1U << command.pin);
                if (command.level) {
                    g_outputShadow |= bit;
                } else {
                    g_outputShadow &= static_cast<uint16_t>(~bit);
                }
                robot::logging::infof("ioexpander", "set pin=%u level=%u shadow=0x%04X",
                                      command.pin,
                                      static_cast<unsigned int>(command.level),
                                      g_outputShadow);
                return writeShadow();
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
