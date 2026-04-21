#include <tasks.h>
#include <Arduino.h>
#include <queues.h>
#include <commands.h>
#include <state.h>
#include <Logger.h>
#include <actions.h>

namespace robot::tasks {
void control_action_task(void* parameter) {
    (void) parameter;

    info("control_action_task", "Task started");

    while (true) {
        Action action;
        if (xQueueReceive(robot::queues::action_command_queue, &action, pdMS_TO_TICKS(100)) == pdPASS) {

            robot::state::setAction(action);

            MotionCommand stopCmd;
            stopCmd.target = {0, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &stopCmd, 0);

            switch (action) {
                case Action::TURN:
                    info("control_action_task", "Handling action command: TURN");
                    robot::actions::turn();
                    break;
                case Action::STOCK:
                    info("control_action_task", "Handling action command: STOCK");
                    robot::actions::stock();
                    break;
                case Action::RELEASE:
                    info("control_action_task", "Handling action command: RELEASE");
                    robot::actions::release();
                    break;
                case Action::TURN_TWO:
                    info("control_action_task", "Handling action command: TURN_TWO");
                    robot::actions::turn_two();
                    break;
                default:
                    warn("control_action_task", "Unknown action command received");
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
}