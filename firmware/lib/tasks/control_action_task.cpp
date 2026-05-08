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
                case Action::TURN_FRONT:
                    info("control_action_task", "Starting TURN action");
                    robot::actions::turn_front();
                    break;
                case Action::STOCK_FRONT:
                    info("control_action_task", "Starting STOCK action");
                    robot::actions::stock_front();
                    break;
                case Action::RELEASE_FRONT:
                    info("control_action_task", "Starting RELEASE action");
                    robot::actions::release_front();
                    break;
                case Action::TURN_TWO_FRONT:
                    info("control_action_task", "Starting TURN_TWO action");
                    robot::actions::turn_two_front();
                    break;
                case Action::TURN_BACK:
                    info("control_action_task", "Starting TURN_BACK action");
                    robot::actions::turn_back();
                    break;
                case Action::STOCK_BACK:
                    info("control_action_task", "Starting STOCK_BACK action");
                    robot::actions::stock_back();
                    break;
                case Action::RELEASE_BACK:
                    info("control_action_task", "Starting RELEASE_BACK action");
                    robot::actions::release_back();
                    break;
                case Action::TURN_TWO_BACK:
                    info("control_action_task", "Starting TURN_TWO_BACK action");
                    robot::actions::turn_two_back();
                    break;
                case Action::IDLE:
                    info("control_action_task", "Going back to IDLE state");
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