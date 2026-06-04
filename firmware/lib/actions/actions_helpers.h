#pragma once

#include <cstdint>
#include <config.h>
#include <state.h>

#include <commands.h>

enum class ArmState: uint8_t {
    IDLE,
    TAKING,
    TURNING,
    DROPPING
};

namespace robot::actions::action_helpers {
    // command helpers
    void performRotation(const PWMControl& actuator_info, uint8_t angle);
    void togglePWM(const PWMControl& actuator_info, bool on);

    // specific actions helpers
    void toggle_pumps_front(uint8_t state);
    void toggle_pumps_back(uint8_t state);
    
    void rotate_turner_front(ArmState state);
    void rotate_turner_back(ArmState state);

    void rotate_grabber_front(GrabberState state);
    void rotate_grabber_back(GrabberState state);

    // misc
    bool mustBeTurned(const ColorResponse& color);
    void endAction();
    uint16_t angleToPWMValue(uint8_t angle);

    bool readColor(robot::config::ColorSensor sensor, ColorResponse& response);
    bool move(float fwd, float strafe, float rotate);
}
