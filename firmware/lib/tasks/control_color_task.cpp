#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <ioexpander.h>
#include <color_sensors.h>

namespace robot::tasks {
void control_color_task(void* parameter) {
    (void)parameter;

    Serial.println("[color] task started");

    while (true) {
        ColorCommand command;
        if ((robot::queues::color_command_queue != nullptr) &&
            (xQueueReceive(robot::queues::color_command_queue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
            if (!robot::color_sensors::apply(command)) {
                Serial.printf("[color] Failed to apply color sensor command\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}