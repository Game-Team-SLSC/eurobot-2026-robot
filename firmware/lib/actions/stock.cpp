#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void stock() {
    MotionCommand mcmd;

    mcmd.target = {-15, 0, 0};

    xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

    vTaskDelay(pdMS_TO_TICKS(300));

    action_helpers::unfold_grabber(true);

    robot::state::setStocking(StockingState::FULL);

    action_helpers::angleTurn(155);

    action_helpers::togglePumps(0b1111);

    vTaskDelay(pdMS_TO_TICKS(500));

    action_helpers::angleTurn(15);

    action_helpers::endAction();
}
}