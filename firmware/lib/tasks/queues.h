#pragma once

#include <FreeRTOS.h>
#include <FreeRTOS/queue.h>
#include <commands.h>

namespace robot::queues {
    extern QueueHandle_t io_command_queue;
    extern QueueHandle_t motion_command_queue;
    extern QueueHandle_t pwm_command_mailbox;
    extern QueueHandle_t action_command_queue;

    void begin();
}

bool enqueuePwmBatch(const PWMCommand& command);