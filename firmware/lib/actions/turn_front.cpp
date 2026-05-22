#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>

namespace robot::actions {
void turn_front() {
    action_helpers::rotate_grabber_front(true);

    GlobalState state = robot::state::get();

    if (state.front_stocking_state == StockingState::FULL || state.front_stocking_state == StockingState::HALF) {
        // check colors
        ColorCommand erColorCmd;
        erColorCmd.sensor = robot::config::ColorSensor::FER;
        xQueueSend(robot::queues::color_command_queue, &erColorCmd, 0);

        ColorResponse erColorResp{};
        const bool erOk = xQueueReceive(robot::queues::color_response_queue, &erColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand irColorCmd;
        irColorCmd.sensor = robot::config::ColorSensor::FIR;
        xQueueSend(robot::queues::color_command_queue, &irColorCmd, 0);

        ColorResponse irColorResp{};
        const bool irOk = xQueueReceive(robot::queues::color_response_queue, &irColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand elColorCmd;
        elColorCmd.sensor = robot::config::ColorSensor::FEL;
        xQueueSend(robot::queues::color_command_queue, &elColorCmd, 0);

        ColorResponse elColorResp{};
        const bool elOk = xQueueReceive(robot::queues::color_response_queue, &elColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand ilColorCmd;
        ilColorCmd.sensor = robot::config::ColorSensor::FIL;
        xQueueSend(robot::queues::color_command_queue, &ilColorCmd, 0);

        ColorResponse ilColorResp{};
        const bool ilOk = xQueueReceive(robot::queues::color_response_queue, &ilColorResp, pdMS_TO_TICKS(1500)) == pdPASS;
        
        // put the right mask on pumps
        uint8_t pumpsMask = 0;
        
        if (erOk && action_helpers::mustBeTurned(erColorResp)) {
            pumpsMask |= 1 << 0;
        }
        if (irOk && action_helpers::mustBeTurned(irColorResp)) {
            pumpsMask |= 1 << 1;
        }
        if (elOk && action_helpers::mustBeTurned(elColorResp)) {
            pumpsMask |= 1 << 2;
        }
        if (ilOk && action_helpers::mustBeTurned(ilColorResp)) {
            pumpsMask |= 1 << 3;
        }
        uint8_t countToTurn = action_helpers::mustBeTurned(erColorResp) + action_helpers::mustBeTurned(irColorResp) + action_helpers::mustBeTurned(elColorResp) + action_helpers::mustBeTurned(ilColorResp);

        
        if (countToTurn == 4) {
            // tout tourner
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(pdMS_TO_TICKS(50));
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(50);
        } else if (countToTurn == 0) {
            // ne rien tourner
            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(250);
            action_helpers::toggle_pumps_front(0b0000);
            vTaskDelay(100);
            action_helpers::rotate_turner_front(ArmState::IDLE);
        } else {
            //
            action_helpers::rotate_turner_front(ArmState::DROPPING);
            vTaskDelay(pdMS_TO_TICKS(50));
            action_helpers::toggle_pumps_front(pumpsMask);
            vTaskDelay(pdMS_TO_TICKS(250));
            action_helpers::rotate_turner_front(ArmState::TURNING);
            vTaskDelay(400);
            action_helpers::toggle_pumps_front(0b0000);
        }

        if (countToTurn != 0) {
            vTaskDelay(pdMS_TO_TICKS(400));
            MotionCommand recule;
            recule.target = {-75, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &recule, 0);

            vTaskDelay(pdMS_TO_TICKS(400));

            MotionCommand ravance;
            ravance.target = {75, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
        }

        // now add the code
    } else {
        MotionCommand mcmd;

        mcmd.target = {-9, 0, 0};

        xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

        vTaskDelay(pdMS_TO_TICKS(300));

        action_helpers::rotate_turner_front(ArmState::TAKING);

        vTaskDelay(pdMS_TO_TICKS(500));

        ColorCommand erColorCmd;
        erColorCmd.sensor = robot::config::ColorSensor::FER;
        xQueueSend(robot::queues::color_command_queue, &erColorCmd, 0);

        ColorResponse erColorResp{};
        const bool erOk = xQueueReceive(robot::queues::color_response_queue, &erColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand irColorCmd;
        irColorCmd.sensor = robot::config::ColorSensor::FIR;
        xQueueSend(robot::queues::color_command_queue, &irColorCmd, 0);

        ColorResponse irColorResp{};
        const bool irOk = xQueueReceive(robot::queues::color_response_queue, &irColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand elColorCmd;
        elColorCmd.sensor = robot::config::ColorSensor::FEL;
        xQueueSend(robot::queues::color_command_queue, &elColorCmd, 0);

        ColorResponse elColorResp{};
        const bool elOk = xQueueReceive(robot::queues::color_response_queue, &elColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand ilColorCmd;
        ilColorCmd.sensor = robot::config::ColorSensor::FIL;
        xQueueSend(robot::queues::color_command_queue, &ilColorCmd, 0);

        ColorResponse ilColorResp{};
        const bool ilOk = xQueueReceive(robot::queues::color_response_queue, &ilColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        bool noneMustBeTurned = true;
        uint8_t pumpsMask = 0;
        if (erOk && action_helpers::mustBeTurned(erColorResp)) {
            pumpsMask |= 1 << 0;
            noneMustBeTurned = false;
        }
        if (irOk && action_helpers::mustBeTurned(irColorResp)) {
            pumpsMask |= 1 << 1;
            noneMustBeTurned = false;
        }
        if (elOk && action_helpers::mustBeTurned(elColorResp)) {
            pumpsMask |= 1 << 2;
            noneMustBeTurned = false;
        }
        if (ilOk && action_helpers::mustBeTurned(ilColorResp)) {
            pumpsMask |= 1 << 3;
            noneMustBeTurned = false;
        }

        if (noneMustBeTurned) {
            action_helpers::toggle_pumps_front(0b0000);

            action_helpers::rotate_turner_front(ArmState::TURNING);
        } else {
            action_helpers::toggle_pumps_front(pumpsMask);
    
            vTaskDelay(pdMS_TO_TICKS(250));
    
            action_helpers::rotate_turner_front(ArmState::TURNING);
            
            vTaskDelay(pdMS_TO_TICKS(750));
    
            action_helpers::toggle_pumps_front(0b0000);
    
            vTaskDelay(pdMS_TO_TICKS(300));
            MotionCommand recule;
            recule.target = {-50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &recule, 0);

            vTaskDelay(pdMS_TO_TICKS(400));

            MotionCommand ravance;
            ravance.target = {50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
        }
    }

    robot::state::setFrontStocking(StockingState::EMPTY);

    action_helpers::endAction();
}
}