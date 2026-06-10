#pragma once

#include <FreeRTOS.h>
#include <freertos/queue.h>

namespace robot::tasks {
    void comm_task(void* parameter);
    void control_task(void* parameter);
    void move_task(void* parameter);
    void control_IO_task(void* parameter);
    void control_PWM_task(void* parameter);
    void control_action_task(void* parameter);
    void sense_task(void* parameter);
    void battery_watch_task(void* parameter);
    void control_color_task(void* parameter);
    void control_leds_task(void* parameter);
    void render_ui_task(void* parameter);
    void watch_encoder(void* parameter);
    void status_led_task(void* parameter);

    bool begin();
};