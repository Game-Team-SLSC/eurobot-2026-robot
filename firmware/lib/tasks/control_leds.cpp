#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <state.h>
#include <Logger.h>

namespace {
uint16_t step_toward(uint16_t current, uint16_t target, uint16_t step) {
    if (current < target) {
        const uint32_t next = static_cast<uint32_t>(current) + step;
        return (next > target) ? target : static_cast<uint16_t>(next);
    }

    if (current > target) {
        return (current - target > step) ? static_cast<uint16_t>(current - step) : target;
    }

    return current;
}
}


namespace robot::tasks {
void control_leds_task(void* parameter) {
    (void)parameter;

    info("control_leds_task", "Task started");

    uint16_t current_red = 0, current_green = 0, current_blue = 0;
    uint16_t target_red = 0, target_green = 0, target_blue = 0;
    
    constexpr TickType_t update_period_ms = 100; // 50Hz
    constexpr uint16_t led_max = 4095;
    constexpr uint16_t fade_step = 700;

    while (true) {
        GlobalState state = robot::state::get();
        // Update target colors based on team
        if (state.isYellowTeam) {
            target_red = 4095;
            target_green = 2500;
            target_blue = 0;
        } else {
            target_red = 0;
            target_green = 0;
            target_blue = 4095;
        }

        current_red = step_toward(current_red, target_red, fade_step);
        current_green = step_toward(current_green, target_green, fade_step);
        current_blue = step_toward(current_blue, target_blue, fade_step);

        PWMCommand red{.controller = robot::config::led_left_r.controller, .pin = robot::config::led_left_r.pin, .value = current_red};
        PWMCommand green{.controller = robot::config::led_left_g.controller, .pin = robot::config::led_left_g.pin, .value = current_green};
        PWMCommand blue{.controller = robot::config::led_left_b.controller, .pin = robot::config::led_left_b.pin, .value = current_blue};

        xQueueSend(robot::queues::pwm_command_queue, &red, 0);
        xQueueSend(robot::queues::pwm_command_queue, &green, 0);
        xQueueSend(robot::queues::pwm_command_queue, &blue, 0);

        vTaskDelay(pdMS_TO_TICKS(update_period_ms));
    }
}
}