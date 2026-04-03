#include <tasks.h>
#include <Arduino.h>

namespace robot::tasks {
    void sense_task(void* parameter) {
        (void)parameter;

        Serial.println("[sense] task started");

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
