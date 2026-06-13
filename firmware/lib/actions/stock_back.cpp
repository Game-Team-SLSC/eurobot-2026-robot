#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void stock_back() {
    GlobalState state = robot::state::get();
    
    if (state.back_grabber_state == GrabberState::CATCHING) {
        action_helpers::rotate_grabber_back(GrabberState::UNFOLDED);
    }

    // move to grab position
    
    action_helpers::move(15, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // grab stack

    action_helpers::rotate_turner_back(ArmState::TAKING);
    action_helpers::toggle_pumps_back(0b1111);
    robot::state::setBackStocking(StockingState::FULL);
    vTaskDelay(pdMS_TO_TICKS(500));

    // retract arm

    action_helpers::rotate_turner_back(ArmState::TURNING);
    
    action_helpers::endAction();
    
    // retract grabber
    vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(4)));
    action_helpers::rotate_grabber_back(GrabberState::FOLDED);
}
}