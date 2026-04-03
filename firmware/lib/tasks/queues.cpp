#include <queues.h>

namespace robot::queues {
    QueueHandle_t io_command_queue = nullptr;
    QueueHandle_t motion_command_queue = nullptr;
    QueueHandle_t pwm_command_mailbox = nullptr;
    QueueHandle_t action_command_queue = nullptr;

    void begin() {
        io_command_queue = xQueueCreate(16, sizeof(IOExpanderCommand));
        motion_command_queue = xQueueCreate(16, sizeof(MotionCommand));
        pwm_command_mailbox = xQueueCreate(16, sizeof(CommandBatch<PWMCommand>));
        action_command_queue = xQueueCreate(1, sizeof(config::Action));
    }
}