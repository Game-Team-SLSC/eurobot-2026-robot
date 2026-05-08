#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void turn_two() {
    CommandBatch<PWMCommand> pwmBatch;

    PWMCommand cmd;
    cmd.controller = robot::config::front_grabber_left.controller;
    cmd.pin = robot::config::front_grabber_left.pin;
    cmd.value = robot::actions::detail::angleToPWMValue(97);

    pwmBatch.add(cmd);

    PWMCommand cmd2;

    cmd2.controller = robot::config::front_right_grabber.controller;
    cmd2.pin = robot::config::front_right_grabber.pin;
    cmd2.value = robot::actions::detail::angleToPWMValue(75);

    pwmBatch.add(cmd2);

    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    GlobalState state = robot::state::get();

    if (state.stockingState == StockingState::HALF) {
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
        if (erOk && !(detail::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);
        }
        if (irOk && !(detail::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);
        }
        if (elOk && !(detail::mustBeTurned(elColorResp))) {
            pumpsMask &= ~(1 << 2);
        }
        if (ilOk && !(detail::mustBeTurned(ilColorResp))) {
            pumpsMask &= ~(1 << 3);
        }

        Serial.printf("h : %f, s : %f, v : %f", erColorResp.h, erColorResp.s, erColorResp.v);

        detail::togglePumps(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));
        // go to 140 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch,  140);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
        // wait 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
        // go to 25 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch,  15);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        // release pumps

        vTaskDelay(pdMS_TO_TICKS(300));
        
        detail::togglePumps(0b0000);
        robot::state::setStocking(StockingState::EMPTY);

        // now add the code
    } else if (state.stockingState == StockingState::FULL) {
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
        if (erOk && !(detail::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);
        }
        if (irOk && !(detail::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);
        }

        detail::togglePumps(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));
        // go to 140 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 140);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
        // wait 1500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
        // go to 25 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 15);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        vTaskDelay(pdMS_TO_TICKS(300));
        
        detail::togglePumps(0b1100);
        robot::state::setStocking(StockingState::HALF);
    } else {
        Serial.println("Not stocking, just turning TWO");

        MotionCommand mcmd;

        mcmd.target = {-15, 0, 0};

        xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

        vTaskDelay(pdMS_TO_TICKS(300));

        detail::angleTurn(pwmBatch,  155);

        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

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
        if (erOk && !(detail::mustBeTurned(erColorResp))) {
            pumpsMask &= ~(1 << 0);  // Efface le bit 0 si ce n'est PAS notre équipe
        }
        if (irOk && !(detail::mustBeTurned(irColorResp))) {
            pumpsMask &= ~(1 << 1);  // Efface le bit 1 si ce n'est PAS notre équipe
        }

        Serial.println(pumpsMask, BIN);

        detail::togglePumps(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(300));

        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 15);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        vTaskDelay(pdMS_TO_TICKS(600));

        robot::state::setStocking(StockingState::HALF);

        pumpsMask = 0b1100;

        detail::togglePumps(pumpsMask);

        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 15);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        vTaskDelay(pdMS_TO_TICKS(300));
        MotionCommand recule;
        recule.target = {-50, 0, 0};
        xQueueSend(robot::queues::motion_command_queue, &recule, 0);

        vTaskDelay(pdMS_TO_TICKS(400));

        MotionCommand ravance;
        ravance.target = {50, 0, 0};
        xQueueSend(robot::queues::motion_command_queue, &ravance, 0);
    }


    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}
}