#include <tasks.h>
#include <Arduino.h>
#include <Logger.h>
#include <encoder.h>
#include <state.h>

namespace robot::tasks {
    void watch_encoder(void* parameter) {
        (void)parameter;

        info("watch_encoder", "Task started");

        robot::encoder::Position previousState = robot::encoder::Position::A_LOW_B_LOW;
        int16_t rawPosition = 0;
        robot::encoder::Position lastStable = robot::encoder::Position::A_LOW_B_LOW;

        while (true) {
            robot::encoder::Position state = robot::encoder::read();
            
            if (state == previousState || state == lastStable) {
                continue;
            }

            switch (state) {
                case robot::encoder::Position::A_LOW_B_LOW:
                    if (previousState == robot::encoder::Position::A_LOW_B_HIGH) {
                        rawPosition -= 1;
                    } else if (previousState == robot::encoder::Position::A_HIGH_B_LOW) {
                        rawPosition += 1;
                    }
                    lastStable = state;
                    break;
                case robot::encoder::Position::A_HIGH_B_HIGH:
                    if (previousState == robot::encoder::Position::A_HIGH_B_LOW) {
                        rawPosition -= 1;
                    } else if (previousState == robot::encoder::Position::A_LOW_B_HIGH) {
                        rawPosition += 1;
                    }
                    lastStable = state;
                    break;
                default:
                    break;
                }
                previousState = state;
            robot::state::setEncoder(rawPosition);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
