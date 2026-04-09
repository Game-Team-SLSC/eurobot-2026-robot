#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>

namespace robot::actions {
void turn() {
    CommandBatch<PWMCommand> pwmBatch;

    PWMCommand cmd;
    cmd.controller = robot::config::front_right_grabber.controller;
    cmd.pin = robot::config::front_right_grabber.pin;
    cmd.value = 100;

    pwmBatch.add(cmd);

    PWMCommand cmd2;

    cmd2.controller = robot::config::front_left_grabber.controller;
    cmd2.pin = robot::config::front_left_grabber.pin;
    cmd2.value = 60;

    pwmBatch.add(cmd2);

    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    GlobalState state = robot::state::get();

    if (state.isStocking) {
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
        if (erOk && detail::isOurTeam(erColorResp)) {
            pumpsMask |= 1 << 0;
        }
        if (irOk && detail::isOurTeam(irColorResp)) {
            pumpsMask |= 1 << 1;
        }
        if (elOk && detail::isOurTeam(elColorResp)) {
            pumpsMask |= 1 << 2;
        }
        if (ilOk && detail::isOurTeam(ilColorResp)) {
            pumpsMask |= 1 << 3;
        }

        Serial.printf("h : %f, s : %f, v : %f", erColorResp.h, erColorResp.s, erColorResp.v);

        detail::togglePumps(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));
        // go to 140 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch,  140);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
        // wait 500 ms
        vTaskDelay(pdMS_TO_TICKS(1500));
        // go to 25 with angle turn
        pwmBatch.clear();
        detail::angleTurn(pwmBatch,  25);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        // release pumps

        vTaskDelay(pdMS_TO_TICKS(300));
        
        detail::togglePumps(0b0000);

        // now add the code
    } else {
        Serial.println("Not stocking, just turning");
        detail::angleTurn(pwmBatch,  175);

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

        uint8_t pumpsMask = 0;
        if (erOk && detail::isOurTeam(erColorResp)) {
            pumpsMask |= 1 << 0;
        }
        if (irOk && detail::isOurTeam(irColorResp)) {
            pumpsMask |= 1 << 1;
        }
        if (elOk && detail::isOurTeam(elColorResp)) {
            pumpsMask |= 1 << 2;
        }
        if (ilOk && detail::isOurTeam(ilColorResp)) {
            pumpsMask |= 1 << 3;
        }

        detail::togglePumps(pumpsMask);

        vTaskDelay(pdMS_TO_TICKS(200));

        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 130);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        vTaskDelay(1000);

        pwmBatch.clear();
        detail::angleTurn(pwmBatch, 25);
        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

        vTaskDelay(pdMS_TO_TICKS(300));

        detail::togglePumps(0b0000);
    }

    robot::state::setStocking(false);

    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}
}