#include <Arduino.h>
#include <commands.h>
#include <queues.h>
#include <ioexpander.h>
#include <Logger.h>

namespace robot::tasks {
void control_IO_task(void* parameter) {
    (void)parameter;

    info("control_io_task", "Task started");

    while (true) {
        IOExpanderCommand command;
        if ((robot::queues::io_command_queue != nullptr) &&
            (xQueueReceive(robot::queues::io_command_queue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
            if (!robot::ioexpander::apply(command)) {
                warn("control_io_task", "Failed to use ioexpander");
            }
        }
    }
}
}