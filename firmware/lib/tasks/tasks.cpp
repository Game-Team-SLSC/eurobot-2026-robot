#include <tasks.h>
#include <encoder.h>
#include <Logger.h>

void memory_task(void* param) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        info("memory", "%d bytes free", ESP.getFreeHeap());
    }
}

namespace robot::tasks {
    bool begin() {
        xTaskCreatePinnedToCore(comm_task, "comm_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(move_task, "move_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_IO_task, "control_IO_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_PWM_task, "pwm_control_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(control_action_task, "control_action_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(battery_watch_task, "battery_watch_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(control_color_task, "control_color_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(control_leds_task, "control_leds_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(render_ui_task, "render_ui_task", 4096, nullptr, 1, nullptr, 1);
        xTaskCreatePinnedToCore(watch_encoder, "watch_encoder", 4096, nullptr, 2, nullptr, 0);
        xTaskCreatePinnedToCore(memory_task, "memory_task", 4096, nullptr, 1, nullptr, 0);
        xTaskCreatePinnedToCore(status_led_task, "status_led_task", 4096, nullptr, 1, nullptr, 0);
        return true;
    }
}
