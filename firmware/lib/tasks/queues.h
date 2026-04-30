#pragma once

#include <FreeRTOS.h>
#include <FreeRTOS/queue.h>
#include <commands.h>

namespace robot::queues {
    extern QueueHandle_t io_command_queue;
    extern QueueHandle_t io_response_queue;

    extern QueueHandle_t motion_command_queue;

    extern QueueHandle_t pwm_command_queue;

    extern QueueHandle_t action_command_queue;

    extern QueueHandle_t color_command_queue;
    extern QueueHandle_t color_response_queue;

    extern QueueHandle_t logs_queue;

    void begin();
}

bool enqueuePwmBatch(const PWMCommand& command);