#include <tasks.h>
#include <Arduino.h>
#include <Logger.h>

namespace robot::tasks {
    void sense_task(void* parameter) {
        (void)parameter;

        info("sense_task", "Task started");

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
