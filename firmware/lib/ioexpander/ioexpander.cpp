#include "ioexpander.h"

#include <TCA9555.h>

#include <buses.h>
#include <config.h>

namespace {
TCA9555 g_tca(robot::config::tca9555_i2c_config.address,
               robot::buses::get(robot::config::tca9555_i2c_config.busId));

uint16_t g_outputShadow = 0;
bool g_ready = false;

bool writeShadow() {
    g_tca.write16(g_outputShadow);
    return true;
}
} // namespace

namespace robot::ioexpander {
    bool begin() {
        if (!g_tca.begin()) {
            return false;
        }

        if (!g_tca.isConnected()) {
            return false;
        }

        for (uint8_t pin = 0; pin < 16; ++pin) {
            if (!g_tca.pinMode1(pin, OUTPUT)) {
                return false;
            }
            if (!g_tca.write1(pin, LOW)) {
                return false;
            }
        }

        g_outputShadow = 0;
        g_ready = true;
        return true;
    }

    bool apply(const Command& command) {
        if (!g_ready) {
            return false;
        }

        switch (command.type) {
            case CommandType::SetPin: {
                if (command.pin >= 16) {
                    return false;
                }

                const uint16_t bit = static_cast<uint16_t>(1U << command.pin);
                if (command.level) {
                    g_outputShadow |= bit;
                } else {
                    g_outputShadow &= static_cast<uint16_t>(~bit);
                }
                return writeShadow();
            }

            case CommandType::WriteMasked: {
                g_outputShadow = static_cast<uint16_t>((g_outputShadow & ~command.mask) |
                                                       (command.value & command.mask));
                return writeShadow();
            }

            default:
                return false;
        }
    }

    uint16_t outputShadow() {
        return g_outputShadow;
    }
}
