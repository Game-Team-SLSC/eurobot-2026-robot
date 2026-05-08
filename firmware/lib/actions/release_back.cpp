#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>

namespace robot::actions {
void release_back() {
    robot::state::setFrontStocking(StockingState::EMPTY);
    action_helpers::rotate_turner_back(140);

    vTaskDelay(pdMS_TO_TICKS(700));
    
    action_helpers::toggle_pumps_back(0b0000);

    vTaskDelay(pdMS_TO_TICKS(200));

    action_helpers::rotate_turner_back(25);

    action_helpers::endAction();
}
}