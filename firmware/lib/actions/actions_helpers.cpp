#include <actions_helpers.h>
#include <Arduino.h>

#include <config.h>
#include <queues.h>
#include <state.h>

namespace {
constexpr uint8_t PUMPS_ALL_MASK = 0b1111;
}

namespace robot::actions::detail {
void togglePumps(uint8_t state) {
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

    CommandBatch<PWMCommand> batch{};

    PWMCommand cmd6;
    cmd6.controller = robot::config::front_interior_left_ev.controller;
    cmd6.pin = robot::config::front_interior_left_ev.pin;
    cmd6.value = (state & (1 << 0)) != 0 ? 4095 : 0;
    
    PWMCommand cmd7;
    cmd7.controller = robot::config::front_interior_right_ev.controller;
    cmd7.pin = robot::config::front_interior_right_ev.pin;
    cmd7.value = (state & (1 << 1)) != 0 ? 4095 : 0;
    
    PWMCommand cmd8;
    cmd8.controller = robot::config::front_exterior_left_ev.controller;
    cmd8.pin = robot::config::front_exterior_left_ev.pin;
    cmd8.value = (state & (1 << 2)) != 0 ? 4095 : 0;
    
    PWMCommand cmd9;
    cmd9.controller = robot::config::front_exterior_right_ev.controller;
    cmd9.pin = robot::config::front_exterior_right_ev.pin;
    cmd9.value = (state & (1 << 3)) != 0 ? 4095 : 0;

    batch.add(cmd6);
    batch.add(cmd7);
    batch.add(cmd8);
    batch.add(cmd9);

    xQueueSend(robot::queues::pwm_command_queue, &batch, 0);


    xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
}

void angleTurn(CommandBatch<PWMCommand>& batch, uint8_t angle) {
    PWMCommand cmd;
    cmd.controller = robot::config::front_left_turner.controller;
    cmd.pin = robot::config::front_left_turner.pin;
    cmd.value = robot::actions::detail::angleToPWMValue(angle);
    batch.add(cmd);

    PWMCommand cmd2;
    cmd2.controller = robot::config::front_right_turner.controller;
    cmd2.pin = robot::config::front_right_turner.pin;
    cmd2.value = robot::actions::detail::angleToPWMValue(180 - angle + 3);
    batch.add(cmd2);
}

uint16_t angleToPWMValue(uint8_t angle) {
    // Map 0-180 to 115-545
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