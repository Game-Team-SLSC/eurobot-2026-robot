#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void stock_back() {
    MotionCommand mcmd;

    mcmd.target = {15, 0, 0};

    xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

    vTaskDelay(pdMS_TO_TICKS(300));

    action_helpers::rotate_grabber_back(true);

    robot::state::setBackStocking(StockingState::FULL);

    action_helpers::rotate_turner_back(ArmState::TAKING);

    action_helpers::toggle_pumps_back(0b1111);

    vTaskDelay(pdMS_TO_TICKS(500));

    action_helpers::rotate_turner_back(ArmState::IDLE);

    action_helpers::endAction();
}
}