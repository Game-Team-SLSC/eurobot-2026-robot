#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>


namespace robot::tasks {
void control_leds_task(void* parameter) {
    (void)parameter;

    info("control_leds_task", "task started");

    uint8_t current_red = 0, current_green = 0, current_blue = 0;
    uint8_t target_red = 0, target_green = 0, target_blue = 0;
    
    constexpr TickType_t update_period_ms = 100; // 10Hz
    TickType_t last_update_time = 0;
    
    while (true) {
        GlobalState state = robot::state::get();
        TickType_t current_time = xTaskGetTickCount();

        // Update target colors based on team
        if (state.isYellowTeam) {
            target_red = 255;
            target_green = 255;
            target_blue = 0;
        } else {
            target_red = 0;
            target_green = 0;
            target_blue = 255;
        }

        // Only update at 10Hz
        if (current_time - last_update_time >= pdMS_TO_TICKS(update_period_ms)) {
            last_update_time = current_time;
            
            // Smooth transition with small steps
            constexpr uint8_t step = 10;
            
            if (current_red < target_red) {
                current_red = (current_red + step > target_red) ? target_red : current_red + step;
            } else if (current_red > target_red) {
                current_red = (current_red - step < target_red) ? target_red : current_red - step;
            }
            
            if (current_green < target_green) {
                current_green = (current_green + step > target_green) ? target_green : current_green + step;
            } else if (current_green > target_green) {
                current_green = (current_green - step < target_green) ? target_green : current_green - step;
            }
            
            if (current_blue < target_blue) {
                current_blue = (current_blue + step > target_blue) ? target_blue : current_blue + step;
            } else if (current_blue > target_blue) {
                current_blue = (current_blue - step < target_blue) ? target_blue : current_blue - step;
            }
            
            CommandBatch<PWMCommand> pwmBatch;
            
            PWMCommand red{.controller = robot::config::PWMController::MISC, .pin = 4, .value = current_red};
            PWMCommand green{.controller = robot::config::PWMController::MISC, .pin = 5, .value = current_green};
            PWMCommand blue{.controller = robot::config::PWMController::MISC, .pin = 6, .value = current_blue};
            
            pwmBatch.add(red);
            pwmBatch.add(green);
            pwmBatch.add(blue);
            
            xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
        }
    }
}
}