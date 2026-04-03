#include <tasks.h>

namespace robot::tasks {
    bool begin() {
        xTaskCreatePinnedToCore(comm_task, "commTask", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(move_task, "moveTask", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_IO_task, "control_IO_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_PWM_task, "miscControl", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_action_task, "control_action_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(sense_task, "sense_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(battery_watch_task, "battery_watch_task", 4096, nullptr, 1, nullptr, 1);

        return true;
    }
}
