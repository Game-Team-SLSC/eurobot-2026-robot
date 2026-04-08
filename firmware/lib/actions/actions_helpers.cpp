#include <actions_helpers.h>

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

    xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
    xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
}

void angleTurn(CommandBatch<PWMCommand>& batch, uint8_t angle) {
    PWMCommand cmd;
    cmd.controller = robot::config::front_left_turner.controller;
    cmd.pin = robot::config::front_left_turner.pin;
    cmd.value = angle;
    batch.add(cmd);

    PWMCommand cmd2;
    cmd2.controller = robot::config::front_right_turner.controller;
    cmd2.pin = robot::config::front_right_turner.pin;
    cmd2.value = 180 - angle + 5;
    batch.add(cmd2);
}

bool isOurTeam(const ColorResponse& color) {
    if (color.s < 0.2f) {
        return false;
    }

    const GlobalState state = robot::state::get();

    if (state.isYellowTeam && (color.h >= 60.0f) && (color.h <= 80.0f)) {
        return true;
    }

    if (!state.isYellowTeam && (color.h >= 210.0f) && (color.h <= 230.0f)) {
        return true;
    }

    return false;
}
}