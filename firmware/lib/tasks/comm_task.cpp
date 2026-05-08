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
    bool back_grabber_folded, front_left_grabber_folded = false;
    
    TickType_t lastLogTime = xTaskGetTickCount();

    while (true) {
        RemoteData data;
        GlobalState currentState = state::get();
        if (robot::remote::fetch(data)) {
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

                    if (currentState.action == Action::TURN) continue;

                    const Action action = Action::TURN;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::stock_action_front_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::STOCK) continue;

                    const Action action = Action::STOCK;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::turn_two_action_front_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    if (currentState.action == Action::TURN_TWO) continue;

                    const Action action = Action::TURN_TWO;
                    currentState.action = action;
                    xQueueSend(robot::queues::action_command_queue, &action, 0);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::toggle_low_speed_btn)) {
                    state::setLowSpeedMode(data.buttons[btnIdx]);
                }

                if (btnIdx == static_cast<uint8_t>(robot::config::toggle_front_grabber_btn)) {
                    if (!data.buttons[btnIdx]) {
                        continue;
                    }

                    front_left_grabber_folded = !front_left_grabber_folded;

                    // Use action_helpers to perform rotations for the front grabbers
                    if (front_left_grabber_folded) {
                        robot::actions::action_helpers::unfold_grabber(false);
                    } else {
                        robot::actions::action_helpers::unfold_grabber(true);
                    }
                }
            }
        }

        // Log frame rate every second
        TickType_t currentTime = xTaskGetTickCount();
        if (currentTime - lastLogTime >= pdMS_TO_TICKS(1000)) {
            state::setRadioFrequency(frameCount);
            frameCount = 0;
            lastLogTime = currentTime;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
}
