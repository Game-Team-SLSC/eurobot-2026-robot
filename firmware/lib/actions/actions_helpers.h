#pragma once

#include <cstdint>
#include <config.h>

#include <commands.h>

namespace robot::actions::action_helpers {
    // command helpers
    void performRotation(const PWMControl& actuator_info, uint8_t angle);
    void togglePWM(const PWMControl& actuator_info, bool on);

    // specific actions helpers
    void togglePumps(uint8_t state);
    void angleTurn(uint8_t angle);
    void unfold_grabber(bool unfolded);

    // misc
    bool mustBeTurned(const ColorResponse& color);
    void endAction();
    uint16_t angleToPWMValue(uint8_t angle);
}
