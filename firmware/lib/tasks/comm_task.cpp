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

namespace robot::tasks {
    void comm_task(void* parameter) {
        (void) parameter;
        
        Serial.println("[comm] task started");
        
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
                        Serial.println("Sending action");
                        if (state.action == robot::config::Action::TURN) continue;

                        const uint8_t action = static_cast<uint8_t>(robot::config::Action::TURN);
                        state.action = robot::config::Action::TURN;
                        xQueueSend(robot::queues::action_command_queue, &action, 0);
                    }

                    if (btnIdx == static_cast<uint8_t>(robot::config::Button::DOUBLE_U_BTN)) {
                        Serial.println("Setting low speed mode to " + String(data.buttons[btnIdx]));
                        GlobalState state = state::get();
                        state::setLowSpeedMode(data.buttons[btnIdx]);
                    }
                }
        
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        }
}
