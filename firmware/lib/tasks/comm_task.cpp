#include <tasks.h>
#include <Arduino.h>
#include <freertos/queue.h>
#include <FreeRTOS.h>
#include <RemoteData.h>
#include <commands.h>
#include <Vec3.h>
#include <movers.h>
#include <remote.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>
#include <actions.h>

namespace robot::tasks {
    void comm_task(void* parameter) {
        (void) parameter;
        
        info("comm", "task started");
        
        while (true) {
            RemoteData data;
            if (robot::remote::fetch(data)) {

                GlobalState currentState = state::get();
                state::setRadio(data);
        
                // Actions

                state::setIsYellow(data.isYellow);
                
                for (uint8_t btnIdx = 0; btnIdx < static_cast<uint8_t>(robot::config::Button::_BUTTON_COUNT); ++btnIdx) {
                    if (currentState.remoteData.buttons[btnIdx] == data.buttons[btnIdx]) {
                        continue;
                    }
                    
                    if (btnIdx == static_cast<uint8_t>(robot::config::turn_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        
                        if (currentState.action == Action::TURN) continue;

                        const Action action = Action::TURN;
                        currentState.action = action;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    }
                    
                    if (btnIdx == static_cast<uint8_t>(robot::config::stock_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }

                        if (currentState.action == Action::STOCK) continue;

                        const Action action = Action::STOCK;
                        currentState.action = action;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    }
                    
                    if (btnIdx == static_cast<uint8_t>(robot::config::release_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        
                        if (currentState.action == Action::RELEASE) continue;

                        const Action action = Action::RELEASE;
                        currentState.action = action;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    }

                    if (btnIdx == static_cast<uint8_t>(robot::config::Button::DOUBLE_U_BTN)) {
                        state::setLowSpeedMode(data.buttons[btnIdx]);
                    }

                    if (btnIdx == static_cast<uint8_t>(robot::config::fold_grabber_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }

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
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::deploy_grabber_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        
                        CommandBatch<PWMCommand> pwmBatch;

                        PWMCommand cmd;

                        cmd.controller = robot::config::front_right_grabber.controller;
                        cmd.pin = robot::config::front_right_grabber.pin;
                        cmd.value = 150;

                        pwmBatch.add(cmd);

                        PWMCommand cmd2;

                        cmd2.controller = robot::config::front_left_grabber.controller;
                        cmd2.pin = robot::config::front_left_grabber.pin;
                        cmd2.value = 10;

                        pwmBatch.add(cmd2);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
                }
        
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        }
}}
