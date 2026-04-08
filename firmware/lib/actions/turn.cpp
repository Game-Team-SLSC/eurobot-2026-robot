#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>

namespace robot::actions {
void turn() {
    CommandBatch<PWMCommand> pwmBatch;

    detail::angleTurn(pwmBatch, 158);

    PWMCommand rightGrabber;
    rightGrabber.controller = robot::config::front_right_grabber.controller;
    rightGrabber.pin = robot::config::front_right_grabber.pin;
    rightGrabber.value = 0;
    pwmBatch.add(rightGrabber);

    PWMCommand leftGrabber;
    leftGrabber.controller = robot::config::front_left_grabber.controller;
    leftGrabber.pin = robot::config::front_left_grabber.pin;
    leftGrabber.value = 168;
    pwmBatch.add(leftGrabber);

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

    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}
}