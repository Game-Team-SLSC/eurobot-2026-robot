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
#include <actions_helpers.h>

namespace robot::tasks {
void comm_task(void* parameter) {
    (void) parameter;

    info("comm", "task started");

    uint32_t frameCount = 0;
    uint32_t lastRetryTime = 0;
    uint32_t restart_btn_hold_start = 0;
    bool motorsHadBeenReset = false;
    
    TickType_t lastLogTime = xTaskGetTickCount();

    while (true) {
        RemoteData data;
        GlobalState currentState = state::get();

        while (robot::remote::fetch(data)) {
            frameCount++;

            state::setRadio(data);

            state::setIsYellow(data.isYellow);

            state::setSpeedGain(map(data.slider, 0.0f, 255.0f, robot::config::min_speed_gain * 255, robot::config::max_speed_gain * 255) / 255.0f);

            for (uint8_t btnIdx = 0; btnIdx < static_cast<uint8_t>(robot::config::Button::_BUTTON_COUNT); ++btnIdx) {
                if (currentState.remoteData.buttons[btnIdx] == data.buttons[btnIdx]) {
                    continue;
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::turn_action_front_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::TURN_FRONT) continue;

                    const Action action = Action::TURN_FRONT;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::stock_action_front_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::STOCK_FRONT) continue;

                    const Action action = Action::STOCK_FRONT;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::turn_two_action_front_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::TURN_TWO_FRONT) continue;

                    const Action action = Action::TURN_TWO_FRONT;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::turn_action_back_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::TURN_BACK) continue;

                    const Action action = Action::TURN_BACK;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::stock_action_back_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::STOCK_BACK) continue;

                    const Action action = Action::STOCK_BACK;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::turn_two_action_back_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::TURN_TWO_BACK) continue;

                    const Action action = Action::TURN_TWO_BACK;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::toggle_low_speed_btn)) {
                    if (!data.buttons[btnIdx]) {
                        motorsHadBeenReset = false;
                        restart_btn_hold_start = 0;
                    }

                    if (restart_btn_hold_start == 0) {
                        restart_btn_hold_start = millis();
                    }

                    if (!motorsHadBeenReset) {
                        robot::movers::resetDrivers();
                        motorsHadBeenReset = true;
                    }

                    if (millis() - restart_btn_hold_start > 2000) {
                        ESP.restart();
                    }
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::toggle_front_grabber_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    // Use action_helpers to perform rotations for the front grabbers
                    if (currentState.front_grabber_state == GrabberState::UNFOLDED) {
                        robot::actions::action_helpers::rotate_grabber_front(GrabberState::CATCHING);
                    } else {
                        robot::actions::action_helpers::rotate_grabber_front(GrabberState::UNFOLDED);
                    }
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::toggle_back_grabber_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    // Use action_helpers to perform rotations for the back grabbers
                    if (currentState.back_grabber_state == GrabberState::UNFOLDED) {
                        robot::actions::action_helpers::rotate_grabber_back(GrabberState::CATCHING);
                    } else {
                        robot::actions::action_helpers::rotate_grabber_back(GrabberState::UNFOLDED);
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(0.1));
        }

        // Log frame rate every second
        TickType_t currentTime = xTaskGetTickCount();
        if (currentTime - lastLogTime >= pdMS_TO_TICKS(1000)) {
            state::setRadioFrequency(frameCount);
            frameCount = 0;
            lastLogTime = currentTime;
        }

        if (millis() - currentState.lastFrameReceivedAt > robot::config::rf_timeout_ms) {
            robot::state::markRadioDisconnected();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
}
