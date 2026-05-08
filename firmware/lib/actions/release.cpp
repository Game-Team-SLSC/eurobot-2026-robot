#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>

namespace robot::actions {
void release() {
    robot::state::setStocking(StockingState::EMPTY);
    action_helpers::angleTurn(140);

    vTaskDelay(pdMS_TO_TICKS(700));
    
    action_helpers::togglePumps(0b0000);

    vTaskDelay(pdMS_TO_TICKS(200));

    action_helpers::angleTurn(25);

    action_helpers::endAction();
}
}