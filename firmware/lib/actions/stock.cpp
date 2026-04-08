#include <actions.h>
#include <actions_helpers.h>

#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>

namespace robot::actions {
void stock() {
    CommandBatch<PWMCommand> pwmBatch;

    robot::state::setStocking(true);

    detail::angleTurn(pwmBatch, 158);
    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    detail::togglePumps(0b1111);

    vTaskDelay(pdMS_TO_TICKS(700));

    pwmBatch.clear();
    detail::angleTurn(pwmBatch, 25);
    xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

    const Action idleAction = Action::IDLE;
    xQueueSend(robot::queues::action_command_queue, &idleAction, 0);
}
}