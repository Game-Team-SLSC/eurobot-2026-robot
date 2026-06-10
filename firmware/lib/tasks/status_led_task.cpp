#include <tasks.h>
#include <Logger.h>
#include <commands.h>
#include <config.h>
#include <queues.h>

namespace robot::tasks {

    void status_led_task(void* param) {
        IOExpanderCommand cmd{.pin = robot::config::run_led_pin, .level = true};
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            cmd.level = true;
            xQueueSend(robot::queues::io_command_queue, &cmd, 0);
            
            vTaskDelay(pdMS_TO_TICKS(1000));
            cmd.level = false;
            xQueueSend(robot::queues::io_command_queue, &cmd, 0);
        }
    }
}