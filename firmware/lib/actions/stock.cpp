#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <actions_helpers.h>

namespace robot::actions {
void stock() {
    CommandBatch<PWMCommand> pwmBatch;

    MotionCommand mcmd;

    mcmd.target = {-15, 0, 0};

    xQueueSend(robot::queues::motion_command_queue, &mcmd, 0);

    vTaskDelay(pdMS_TO_TICKS(300));

    PWMCommand cmd;
    cmd.controller = robot::config::front_right_grabber.controller;
    cmd.pin = robot::config::front_right_grabber.pin;
    cmd.value = robot::actions::detail::angleToPWMValue(97);

    pwmBatch.add(cmd);

    PWMCommand cmd2;

    cmd2.controller = robot::config::front_left_grabber.controller;
    cmd2.pin = robot::config::front_left_grabber.pin;
    cmd2.value = robot::actions::detail::angleToPWMValue(75);

    pwmBatch.add(cmd2);

    robot::state::setStocking(StockingState::FULL);

    detail::angleTurn(pwmBatch, 155);
    
    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    detail::togglePumps(0b1111);

    vTaskDelay(pdMS_TO_TICKS(700));

    pwmBatch.clear();
    detail::angleTurn(pwmBatch, 15);
    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}
}