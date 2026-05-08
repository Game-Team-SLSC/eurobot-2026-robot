#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void turn_two_front() {
    action_helpers::rotate_grabber_front(true);

    GlobalState state = robot::state::get();

    if (state.front_stocking_state == StockingState::HALF) {
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
        uint8_t pumpsMask = 0b1111;
        if (erOk && !(action_helpers::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);
        }
        if (irOk && !(action_helpers::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);
        }
        if (elOk && !(action_helpers::mustBeTurned(elColorResp))) {
            pumpsMask &= ~(1 << 2);
        }
        if (ilOk && !(action_helpers::mustBeTurned(ilColorResp))) {
            pumpsMask &= ~(1 << 3);
        }

        action_helpers::toggle_pumps_front(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));
        
        action_helpers::rotate_turner_front(140);

        vTaskDelay(pdMS_TO_TICKS(500));
        
        action_helpers::rotate_turner_front(15);

        // release pumps

        vTaskDelay(pdMS_TO_TICKS(300));
        
        action_helpers::toggle_pumps_front(0b0000);
        robot::state::setFrontStocking(StockingState::EMPTY);
    } else if (state.front_stocking_state == StockingState::FULL) {
        // check colors on right side only
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

        // put the right mask on pumps (only right two)
        uint8_t pumpsMask = 0b1111;
        if (erOk && !(action_helpers::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);
        }
        if (irOk && !(action_helpers::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);
        }

        action_helpers::toggle_pumps_front(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));

        action_helpers::rotate_turner_front(140);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        
        action_helpers::rotate_turner_front(15);

        vTaskDelay(pdMS_TO_TICKS(300));
        
        action_helpers::toggle_pumps_front(0b1100);
        robot::state::setFrontStocking(StockingState::HALF);
    } else {
        MotionCommand mcmd;

        mcmd.target = {-15, 0, 0};

        xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

        vTaskDelay(pdMS_TO_TICKS(300));

        action_helpers::rotate_turner_front(155);

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

        uint8_t pumpsMask = 0b1111;
        if (erOk && !(action_helpers::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);  // Efface le bit 0 si ce n'est PAS notre équipe
        }
        if (irOk && !(action_helpers::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);  // Efface le bit 1 si ce n'est PAS notre équipe
        }

        action_helpers::toggle_pumps_front(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(300));

        action_helpers::rotate_turner_front(15);

        vTaskDelay(pdMS_TO_TICKS(600));

        robot::state::setFrontStocking(StockingState::HALF);

        pumpsMask = 0b1100;

        action_helpers::toggle_pumps_front(pumpsMask);

        action_helpers::rotate_turner_front(15);

        vTaskDelay(pdMS_TO_TICKS(300));
        MotionCommand recule;
        recule.target = {-50, 0, 0};
        xQueueSend(robot::queues::motion_command_queue, &recule, 0);

        vTaskDelay(pdMS_TO_TICKS(400));

        MotionCommand ravance;
        ravance.target = {50, 0, 0};
        xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
    }


    action_helpers::endAction();
}
}