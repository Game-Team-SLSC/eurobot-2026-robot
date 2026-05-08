#include <actions_helpers.h>
#include <Arduino.h>

#include <config.h>
#include <queues.h>
#include <state.h>

namespace {
constexpr uint8_t PUMPS_ALL_MASK = 0b1111;
}

namespace robot::actions::action_helpers {
void togglePWM(const PWMControl& actuator_info, bool on) {
    PWMCommand cmd;

    cmd.controller = actuator_info.controller;
    cmd.pin = actuator_info.pin;
    cmd.value = on ? 4095 : 0;

    xQueueSend(robot::queues::pwm_command_queue, &cmd, 0);
}

void togglePumps(uint8_t state) {
    if (state == 0b0000) {
        robot::state::setPumpsStatus(false);
    } else {
        robot::state::setPumpsStatus(true);
    }
    state &= PUMPS_ALL_MASK;

    IOExpanderCommand cmd2;
    cmd2.expander = robot::config::IOExpander::KINETIC;
    cmd2.pin = 7;
    cmd2.level = (state & (1 << 1)) != 0;

    IOExpanderCommand cmd3;
    cmd3.expander = robot::config::IOExpander::KINETIC;
    cmd3.pin = 6;
    cmd3.level = (state & (1 << 0)) != 0;

    IOExpanderCommand cmd4;
    cmd4.expander = robot::config::IOExpander::KINETIC;
    cmd4.pin = 13;
    cmd4.level = (state & (1 << 3)) != 0;

    IOExpanderCommand cmd5;
    cmd5.expander = robot::config::IOExpander::KINETIC;
    cmd5.pin = 12;
    cmd5.level = (state & (1 << 2)) != 0;

    togglePWM(robot::config::front_interior_left_ev, (state & (1 << 0)) != 0);
    togglePWM(robot::config::front_interior_right_ev, (state & (1 << 1)) != 0);
    togglePWM(robot::config::front_exterior_left_ev, (state & (1 << 2)) != 0);
    togglePWM(robot::config::front_exterior_right_ev, (state & (1 << 3)) != 0);

    xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
}

void performRotation(const PWMControl& actuator_info, uint8_t angle) {
    PWMCommand cmd;

    cmd.controller = actuator_info.controller;
    cmd.pin = actuator_info.pin;
    cmd.value = robot::actions::action_helpers::angleToPWMValue(angle);

    xQueueSend(robot::queues::pwm_command_queue, &cmd, 0);
}

void angleTurn(uint8_t angle) {
    performRotation(robot::config::front_left_turner, angle);
    performRotation(robot::config::front_right_turner, 180 - angle + 3);
}

void unfold_grabber(bool unfolded) {
    if (unfolded) {
        performRotation(robot::config::front_grabber_left, 68);
        performRotation(robot::config::front_right_grabber, 75);
    } else {
        performRotation(robot::config::front_grabber_left, 133);
        performRotation(robot::config::front_right_grabber, 10);
    }
}

void endAction() {
    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}

uint16_t angleToPWMValue(uint8_t angle) {
    return map(angle, 0, 180, 115, 545);
}

bool mustBeTurned(const ColorResponse& color) {
    if (color.s < 0.2f) {
        return false;
    }

    const GlobalState state = robot::state::get();

    if (!state.isYellowTeam && (color.h >= 50.0f) && (color.h <= 80.0f)) {
        return true;
    }

    if (state.isYellowTeam && (color.h >= 210.0f) && (color.h <= 230.0f)) {
        return true;
    }

    return false;
}
}