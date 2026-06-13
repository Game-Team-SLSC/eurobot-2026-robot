#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>
#include <Logger.h>

namespace robot::actions {
void turn_two_front() {
    GlobalState state = robot::state::get();

    // grab if needed

    if (state.front_stocking_state == StockingState::EMPTY) {
        action_helpers::move(-9, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
        action_helpers::rotate_turner_front(ArmState::TAKING);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Read sensors
    ColorResponse erColorResp{}, irColorResp{}, elColorResp{}, ilColorResp{};
    action_helpers::readColor(robot::config::ColorSensor::FER, erColorResp);
    action_helpers::readColor(robot::config::ColorSensor::FIR, irColorResp);
    action_helpers::readColor(robot::config::ColorSensor::FEL, elColorResp);
    action_helpers::readColor(robot::config::ColorSensor::FIL, ilColorResp);

    if (state.front_stocking_state == StockingState::HALF) {
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

        info("turn_two_front", "Turning %d stacks from half stockage", countToTurn);

        if (countToTurn == 2) {
            // turn all

            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(pdMS_TO_TICKS(400));

            action_helpers::move(-75, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(400));

            action_helpers::move(75, 0, 0);
        } else if (countToTurn == 1) {
            // turn one

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 1)));
            action_helpers::toggle_pumps_front(0b0000);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);
        } else {
            // turn none

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(450));
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_front(ArmState::IDLE);
        }
        robot::state::setFrontStocking(StockingState::EMPTY);
    } else if (state.front_stocking_state == StockingState::FULL) {
        
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

        info("turn_two_front", "Turning %d stacks from full stockage", countToTurn);

        if (countToTurn == 0) {
            // drop directly

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::TURNING);
        } else if (countToTurn == 2) {
            // turn 2

            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(100));
            action_helpers::toggle_pumps_front(0b1100);
        } else {
            // turn some

            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_front(0b1100);
        }

        if (countToTurn != 0) {
            // go back and forth to unstuck pieces

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);
        }

        robot::state::setFrontStocking(StockingState::HALF);
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

        info("turn_two_front", "Turning %d stacks", countToTurn);

        if (countToTurn == 0) {
            // drop directly
            
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::IDLE);
        }  else if (countToTurn == 2) {
            // turn 2

            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_front(0b1100);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);
        } else {
            // turn some

            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(action_helpers::getDelayForTurn(countToTurn + 2)));
            action_helpers::toggle_pumps_front(0b1100);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(-75, 0, 0);

            vTaskDelay(pdMS_TO_TICKS(400));
            action_helpers::move(75, 0, 0);
        }

        robot::state::setFrontStocking(StockingState::HALF);
    }
    action_helpers::endAction();
}}