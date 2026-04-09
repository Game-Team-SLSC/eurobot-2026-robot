#include <tasks.h>
#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <pwm-controller.h>
#include <Logger.h>

namespace robot::tasks {
    void control_PWM_task(void* parameter) {
        (void) parameter;

        info("control_pwm_task", "Task started");

        while (true) {
            CommandBatch<PWMCommand> pwmBatch{};
            if (xQueueReceive(robot::queues::pwm_command_queue, &pwmBatch, pdMS_TO_TICKS(100)) == pdPASS) {
                info("control_pwm_task", "Handling PWM command batch (count=%u)", static_cast<unsigned int>(pwmBatch.count));
                robot::pwmcontroller::apply(pwmBatch);
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }     
}