#include <tasks.h>
#include <Arduino.h>
#include <queues.h>
#include <commands.h>
#include <state.h>

namespace robot::tasks {
void control_action_task(void* parameter) {
    (void) parameter;

    Serial.println("[control_action_task]: Task started");

    while (true) {
        uint8_t action;
        if (xQueueReceive(robot::queues::action_command_queue, &action, pdMS_TO_TICKS(100)) == pdPASS) {
            robot::state::setAction(static_cast<robot::config::Action>(action));
            switch (static_cast<robot::config::Action>(action)) {
                case robot::config::Action::TURN:
                    {
                        CommandBatch<PWMCommand> pwmBatch;

                        // put in "saisie" position

                        PWMCommand cmd;
                        cmd.controller = robot::config::front_left_turner.controller;
                        cmd.pin = robot::config::front_left_turner.pin;
                        cmd.value = 180;
                        pwmBatch.add(cmd);

                        xQueueSend(robot::queues::pwm_command_mailbox, &pwmBatch, 0);

                        //  pump on

                        IOExpanderCommand cmd2;
                        cmd2.expander = robot::config::IOExpander::KINETIC;
                        cmd2.pin = 7;
                        cmd2.level = true;

                        xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
                        
                        vTaskDelay(pdMS_TO_TICKS(700));
                        
                        pwmBatch.clear();
                        
                        // Retract arm

                        PWMCommand cmd1;
                        cmd1.controller = robot::config::front_left_turner.controller;
                        cmd1.pin = robot::config::front_left_turner.pin;
                        cmd1.value = 25;
                        pwmBatch.add(cmd1);
                        
                        xQueueSend(robot::queues::pwm_command_mailbox, &pwmBatch, 0);

                        vTaskDelay(pdMS_TO_TICKS(100));

                        // pump off

                        cmd2.expander = robot::config::IOExpander::KINETIC;
                        cmd2.pin = 7;
                        cmd2.level = false;

                        xQueueSend(robot::queues::io_command_queue, &cmd2, 0);

                        vTaskDelay(pdMS_TO_TICKS(800));

                        MotionCommand motionCmd;
                        motionCmd.target = {-50, 0, 0};
                        xQueueSend(robot::queues::motion_command_queue, &motionCmd, 0);

                        vTaskDelay(pdMS_TO_TICKS(500));

                        motionCmd.target = {50, 0, 0};
                        xQueueSend(robot::queues::motion_command_queue, &motionCmd, 0);

                        vTaskDelay(pdMS_TO_TICKS(500));

                        uint8_t idleAction = static_cast<uint8_t>(robot::config::Action::IDLE);
                        xQueueSend(robot::queues::action_command_queue, &idleAction, 0);

                        break;
                 }
                default:
                    Serial.println("[control_action_task] Unknown action command received");
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
}