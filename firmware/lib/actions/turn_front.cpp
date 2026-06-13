#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>

namespace robot::actions {
void turn_front() {
    GlobalState state = robot::state::get();

    if (state.front_grabber_state == GrabberState::CATCHING) {
        action_helpers::rotate_grabber_front(GrabberState::UNFOLDED);
    }

    // grab the stack if not already grabbing

    if (state.front_stocking_state == StockingState::EMPTY) {
        action_helpers::move(-9, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        action_helpers::rotate_turner_front(ArmState::TAKING);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // check colors

    ColorResponse erColorResp, irColorResp, elColorResp, ilColorResp;
    
    bool erOk = action_helpers::readColor(robot::config::ColorSensor::FER, erColorResp);
    bool irOk = action_helpers::readColor(robot::config::ColorSensor::FIR, irColorResp);
    bool elOk = action_helpers::readColor(robot::config::ColorSensor::FEL, elColorResp);
    bool ilOk = action_helpers::readColor(robot::config::ColorSensor::FIL, ilColorResp);
    
    // make pumps mask

    uint8_t pumpsMask = 0;
    uint8_t countToTurn = 0;

    if (erOk && action_helpers::mustBeTurned(erColorResp)) {
        pumpsMask |= 1 << 0;
        countToTurn++;
    }
    if (irOk && action_helpers::mustBeTurned(irColorResp)) {
        pumpsMask |= 1 << 1;
        countToTurn++;
    }
    if (elOk && action_helpers::mustBeTurned(elColorResp)) {
        pumpsMask |= 1 << 2;
        countToTurn++;
    }
    if (ilOk && action_helpers::mustBeTurned(ilColorResp)) {
        pumpsMask |= 1 << 3;
        countToTurn++;
    }


    if (state.front_stocking_state == StockingState::FULL || state.front_stocking_state == StockingState::HALF) {
        info("turn_front", "Turning %d stacks from stockage", countToTurn);
        if (countToTurn == 4) {
            // turn all

            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(50));
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else if (countToTurn == 0) {
            // turn nothing

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::IDLE);
        } else {
            //  turn some

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(500));
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn)));
            action_helpers::toggle_pumps_front(0b0000);
        }
    } else {
        info("turn_front", "Turning %d stacks", countToTurn);
        if (countToTurn == 0) {
            // drop directly

            info("turn_front", "");
            action_helpers::toggle_pumps_front(0b0000);
            action_helpers::rotate_turner_front(ArmState::IDLE);
        } else {
            // turn some

            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn)));
            action_helpers::toggle_pumps_front(0b0000);
        }
    }

    if (countToTurn != 0) {
        // go back and forth to help the stacks get out

        vTaskDelay(pdMS_TO_TICKS(400));
        action_helpers::move(-75, 0, 0);

        vTaskDelay(pdMS_TO_TICKS(400));
        action_helpers::move(75, 0, 0);
    }

    robot::state::setFrontStocking(StockingState::EMPTY);

    action_helpers::endAction();
}
}