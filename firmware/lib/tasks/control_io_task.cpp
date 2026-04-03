#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <ioexpander.h>

namespace robot::tasks {
void control_IO_task(void* parameter) {
    (void)parameter;

    Serial.println("[io] task started");

    while (true) {
        IOExpanderCommand command;
        if ((robot::queues::io_command_queue != nullptr) &&
            (xQueueReceive(robot::queues::io_command_queue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
            if (!robot::ioexpander::apply(command)) {
                Serial.printf("[io] Failed to apply ioexpander command to state : %u at level : %u\n", static_cast<unsigned int>(command.pin), static_cast<unsigned int>(command.level));
            }
        }
    }
}
}