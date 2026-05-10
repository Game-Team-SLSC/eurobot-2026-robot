#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>

namespace robot::actions {
void turn_back() {
    action_helpers::rotate_grabber_back(true);

    GlobalState state = robot::state::get();

    if (state.back_stocking_state == StockingState::FULL || state.back_stocking_state == StockingState::HALF) {
        // check colors
        ColorCommand erColorCmd;
        erColorCmd.sensor = robot::config::ColorSensor::BER;
        xQueueSend(robot::queues::color_command_queue, &erColorCmd, 0);

        ColorResponse erColorResp{};
        const bool erOk = xQueueReceive(robot::queues::color_response_queue, &erColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand irColorCmd;
        irColorCmd.sensor = robot::config::ColorSensor::BIR;
        xQueueSend(robot::queues::color_command_queue, &irColorCmd, 0);

        ColorResponse irColorResp{};
        const bool irOk = xQueueReceive(robot::queues::color_response_queue, &irColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand elColorCmd;
        elColorCmd.sensor = robot::config::ColorSensor::BEL;
        xQueueSend(robot::queues::color_command_queue, &elColorCmd, 0);

        ColorResponse elColorResp{};
        const bool elOk = xQueueReceive(robot::queues::color_response_queue, &elColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand ilColorCmd;
        ilColorCmd.sensor = robot::config::ColorSensor::BIL;
        xQueueSend(robot::queues::color_command_queue, &ilColorCmd, 0);

        ColorResponse ilColorResp{};
        const bool ilOk = xQueueReceive(robot::queues::color_response_queue, &ilColorResp, pdMS_TO_TICKS(1500)) == pdPASS;
        
        // put the right mask on pumps
        uint8_t pumpsMask = 0;
        bool noneMustBeTurned = true;
        if (erOk && action_helpers::mustBeTurned(erColorResp)) {
            pumpsMask |= 1 << 0;
            noneMustBeTurned = false;
        }
        if (irOk && action_helpers::mustBeTurned(irColorResp)) {
            pumpsMask |= 1 << 1;
            noneMustBeTurned = false;
        }
        if (elOk && action_helpers::mustBeTurned(elColorResp)) {
            pumpsMask |= 1 << 3;
            noneMustBeTurned = false;
        }
        if (ilOk && action_helpers::mustBeTurned(ilColorResp)) {
            pumpsMask |= 1 << 2;
            noneMustBeTurned = false;
        }
        bool allMustBeTurned = action_helpers::mustBeTurned(erColorResp) && action_helpers::mustBeTurned(irColorResp) && action_helpers::mustBeTurned(elColorResp) && action_helpers::mustBeTurned(ilColorResp);

        action_helpers::toggle_pumps_back(pumpsMask);

        if (allMustBeTurned) {
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_back(0b0000);
            action_helpers::rotate_turner_back(ArmState::TURNING);
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
            // go to 140 with angle turn
            action_helpers::rotate_turner_back(ArmState::TAKING);
            // wait 500 ms
            vTaskDelay(pdMS_TO_TICKS(400));
            // go to 15 with angle turn
            action_helpers::rotate_turner_back(ArmState::TURNING);
        }

        if (noneMustBeTurned) {
            vTaskDelay(pdMS_TO_TICKS(300));
            action_helpers::toggle_pumps_back(0b0000);
            action_helpers::rotate_turner_back(ArmState::TURNING);
        } else {
            action_helpers::toggle_pumps_back(pumpsMask);
    
            vTaskDelay(pdMS_TO_TICKS(100));
    
            action_helpers::rotate_turner_back(ArmState::TURNING);
            
            vTaskDelay(pdMS_TO_TICKS(400));
    
            action_helpers::toggle_pumps_back(0b0000);
    
            vTaskDelay(pdMS_TO_TICKS(300));
            MotionCommand recule;
            recule.target = {50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &recule, 0);

            vTaskDelay(pdMS_TO_TICKS(400));

            MotionCommand ravance;
            ravance.target = {-50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
        }

        // now add the code
    } else {
        MotionCommand mcmd;

        mcmd.target = {15, 0, 0};

        xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

        vTaskDelay(pdMS_TO_TICKS(300));

        action_helpers::rotate_turner_back(ArmState::TAKING);

        vTaskDelay(pdMS_TO_TICKS(500));

        ColorCommand erColorCmd;
        erColorCmd.sensor = robot::config::ColorSensor::BER;
        xQueueSend(robot::queues::color_command_queue, &erColorCmd, 0);

        ColorResponse erColorResp{};
        const bool erOk = xQueueReceive(robot::queues::color_response_queue, &erColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand irColorCmd;
        irColorCmd.sensor = robot::config::ColorSensor::BIR;
        xQueueSend(robot::queues::color_command_queue, &irColorCmd, 0);

        ColorResponse irColorResp{};
        const bool irOk = xQueueReceive(robot::queues::color_response_queue, &irColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand elColorCmd;
        elColorCmd.sensor = robot::config::ColorSensor::BEL;
        xQueueSend(robot::queues::color_command_queue, &elColorCmd, 0);

        ColorResponse elColorResp{};
        const bool elOk = xQueueReceive(robot::queues::color_response_queue, &elColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

        ColorCommand ilColorCmd;
        ilColorCmd.sensor = robot::config::ColorSensor::BIL;
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

        // prints which sensor has detected a color that must be turned
        if (erOk) {
            info("turn_back", "ER color: h=%.1f s=%.1f v=%.1f -> %s", erColorResp.h, erColorResp.s, erColorResp.v, action_helpers::mustBeTurned(erColorResp) ? "TURN" : "NO TURN");
        }
        if (irOk) {
            info("turn_back", "IR color: h=%.1f s=%.1f v=%.1f -> %s", irColorResp.h, irColorResp.s, irColorResp.v, action_helpers::mustBeTurned(irColorResp) ? "TURN" : "NO TURN");
        }
        if (elOk) {
            info("turn_back", "EL color: h=%.1f s=%.1f v=%.1f -> %s", elColorResp.h, elColorResp.s, elColorResp.v, action_helpers::mustBeTurned(elColorResp) ? "TURN" : "NO TURN");
        }
        if (ilOk) {
            info("turn_back", "IL color: h=%.1f s=%.1f v=%.1f -> %s", ilColorResp.h, ilColorResp.s, ilColorResp.v, action_helpers::mustBeTurned(ilColorResp) ? "TURN" : "NO TURN");
        }

        if (noneMustBeTurned) {
            action_helpers::toggle_pumps_back(0b0000);

            action_helpers::rotate_turner_back(ArmState::TURNING);
        } else {
            action_helpers::toggle_pumps_back(pumpsMask);
    
            vTaskDelay(pdMS_TO_TICKS(100));
    
            action_helpers::rotate_turner_back(ArmState::TURNING);
            
            vTaskDelay(pdMS_TO_TICKS(400));
    
            action_helpers::toggle_pumps_back(0b0000);
    
            vTaskDelay(pdMS_TO_TICKS(300));
            MotionCommand recule;
            recule.target = {50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &recule, 0);

            vTaskDelay(pdMS_TO_TICKS(400));

            MotionCommand ravance;
            ravance.target = {-50, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
        }
    }

    robot::state::setBackStocking(StockingState::EMPTY);

    action_helpers::endAction();
}
}