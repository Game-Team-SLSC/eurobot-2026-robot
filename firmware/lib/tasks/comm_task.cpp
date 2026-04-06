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

namespace robot::tasks {
    void comm_task(void* parameter) {
        (void) parameter;
        
        info("comm", "task started");
        
        while (true) {
            RemoteData data;
            if (robot::remote::fetch(data)) {

                state::setRadio(data);
        
                // Actions
                
                for (uint8_t btnIdx = 0; btnIdx < static_cast<uint8_t>(robot::config::Button::_BUTTON_COUNT); ++btnIdx) {
                    if (btnIdx == static_cast<uint8_t>(robot::config::turn_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        GlobalState state = state::get();
                        if (state.action == robot::config::Action::TURN) continue;

                        const uint8_t action = static_cast<uint8_t>(robot::config::Action::TURN);
                        state.action = robot::config::Action::TURN;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::stock_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        GlobalState state = state::get();
                        if (state.action == robot::config::Action::STOCK) continue;

                        const uint8_t action = static_cast<uint8_t>(robot::config::Action::STOCK);
                        state.action = robot::config::Action::STOCK;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::release_action_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        GlobalState state = state::get();
                        if (state.action == robot::config::Action::RELEASE) continue;

                        const uint8_t action = static_cast<uint8_t>(robot::config::Action::RELEASE);
                        state.action = robot::config::Action::RELEASE;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::yellow_mode_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        GlobalState state = state::get();
                        state::setIsYellow(true);
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::blue_mode_btn)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        GlobalState state = state::get();
                        state::setIsYellow(false);
                    }

                    if (btnIdx == static_cast<uint8_t>(robot::config::Button::DOUBLE_U_BTN)) {
                        GlobalState state = state::get();
                        state::setLowSpeedMode(data.buttons[btnIdx]);
                    }

                    if (btnIdx == static_cast<uint8_t>(robot::config::Button::RSIDE_R_BTN)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }

                        CommandBatch<PWMCommand> pwmBatch;

                        PWMCommand cmd;
                        cmd.controller = robot::config::front_right_grabber.controller;
                        cmd.pin = robot::config::front_right_grabber.pin;
                        cmd.value = 50;

                        pwmBatch.add(cmd);

                        PWMCommand cmd2;

                        cmd2.controller = robot::config::front_left_grabber.controller;
                        cmd2.pin = robot::config::front_left_grabber.pin;
                        cmd2.value = 118;

                        pwmBatch.add(cmd2);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
                    } else if (btnIdx == static_cast<uint8_t>(robot::config::Button::RSIDE_D_BTN)) {
                        if (!data.buttons[btnIdx]) {
                            continue;
                        }
                        
                        CommandBatch<PWMCommand> pwmBatch;

                        PWMCommand cmd;

                        cmd.controller = robot::config::front_right_grabber.controller;
                        cmd.pin = robot::config::front_right_grabber.pin;
                        cmd.value = 0;

                        pwmBatch.add(cmd);

                        PWMCommand cmd2;

                        cmd2.controller = robot::config::front_left_grabber.controller;
                        cmd2.pin = robot::config::front_left_grabber.pin;
                        cmd2.value = 168;

                        pwmBatch.add(cmd2);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
                }
        
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        }
}}
