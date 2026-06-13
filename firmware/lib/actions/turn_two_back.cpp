#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>
#include <Logger.h>

namespace robot::actions {
void turn_two_back() {
    GlobalState state = robot::state::get();

    // grab if needed

    if (state.back_stocking_state == StockingState::EMPTY) {
        action_helpers::move(9, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        action_helpers::rotate_turner_back(ArmState::TAKING);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Read sensors
    ColorResponse erColorResp{}, irColorResp{}, elColorResp{}, ilColorResp{};
    action_helpers::readColor(robot::config::ColorSensor::BER, erColorResp);
    action_helpers::readColor(robot::config::ColorSensor::BIR, irColorResp);
    action_helpers::readColor(robot::config::ColorSensor::BEL, elColorResp);
    action_helpers::readColor(robot::config::ColorSensor::BIL, ilColorResp);

    if (state.back_stocking_state == StockingState::HALF) {
        
        // make mask

        uint8_t pumpsMask = 0b0000;
        uint8_t countToTurn = 0;

        if (action_helpers::mustBeTurned(elColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 2;
        }
        if (action_helpers::mustBeTurned(ilColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 3;
        }
        info("turn_two_back", "Turning %d stacks from half stockage", countToTurn);
        
        if (countToTurn == 2) {
            // turn all

            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            action_helpers::toggle_pumps_back(0b0000);
            vTaskDelay(pdMS_TO_TICKS(400));

            action_helpers::move(75, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(400));

            action_helpers::move(-75, 0, 0);
        } else if (countToTurn == 1) {
            // turn one

            action_helpers::rotate_turner_back(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 1)));
            action_helpers::toggle_pumps_back(0b0000);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);
        } else {
            // turn none

            action_helpers::rotate_turner_back(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(450));
            action_helpers::toggle_pumps_back(0b0000);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_back(ArmState::IDLE);
        }
        robot::state::setBackStocking(StockingState::EMPTY);
    } else if (state.back_stocking_state == StockingState::FULL) {
        
        // make mask

        uint8_t pumpsMask = 0b1100;
        uint8_t countToTurn = 0;

        if (action_helpers::mustBeTurned(erColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 0;
        }
        if (action_helpers::mustBeTurned(irColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 1;  
        }

        info("turn_two_back", "Turning %d stacks from full stockage", countToTurn);

        if (countToTurn == 0) {
            // drop directly

            action_helpers::rotate_turner_back(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_back(ArmState::TURNING);
        } else if (countToTurn == 2) {
            // turn 2

            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(100));
            action_helpers::toggle_pumps_back(0b1100);
        } else {
            // turn some

            action_helpers::rotate_turner_back(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_back(0b1100);
        }

        if (countToTurn != 0) {
            // go back and forth to unstuck pieces

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);
        }
        robot::state::setBackStocking(StockingState::HALF);
    } else {
        // make mask

        uint8_t pumpsMask = 0b1100;
        uint8_t countToTurn = 0;

        if (action_helpers::mustBeTurned(erColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 0;  
        }
        if (action_helpers::mustBeTurned(irColorResp)) {
            countToTurn++;
            pumpsMask |= 1 << 1;  
        }

        info("turn_two_back", "Turning %d stacks", countToTurn);

        if (countToTurn == 0) {
            // drop directly
            
            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_back(ArmState::IDLE);
        }  else if (countToTurn == 2) {
            // turn 2

            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_back(0b1100);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);
        } else {
            // turn some

            action_helpers::toggle_pumps_back(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_back(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_back(0b1100);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);
        }

        robot::state::setBackStocking(StockingState::HALF);
    }
    action_helpers::endAction();

}}