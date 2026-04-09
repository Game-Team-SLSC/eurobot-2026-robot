#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <ioexpander.h>
#include <color_sensors.h>
#include <Logger.h>

namespace robot::tasks {
void control_color_task(void* parameter) {
    (void)parameter;

    info("control_color_task", "task started");

    while (true) {
        ColorCommand command;
        if ((robot::queues::color_command_queue != nullptr) &&
            (xQueueReceive(robot::queues::color_command_queue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
            info("control_color_task", "Handling color command (sensor=%u)", static_cast<unsigned int>(command.sensor));
            if (!robot::color_sensors::apply(command)) {
                warn("control_color_task", "Failed to apply color sensor command");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}