#include <tasks.h>
#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <pwm-controller.h>

namespace robot::tasks {
    void control_PWM_task(void* parameter) {
        (void) parameter;

        Serial.println("[misc]: Task started");

        while (true) {
            CommandBatch<PWMCommand> pwmBatch{};
            if (xQueueReceive(robot::queues::pwm_command_mailbox, &pwmBatch, pdMS_TO_TICKS(100)) == pdPASS) {
                robot::pwmcontroller::apply(pwmBatch);
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }     
}