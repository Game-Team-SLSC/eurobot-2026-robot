#include <tasks.h>
#include <Arduino.h>
#include <battery.h>
#include <config.h>
#include <queues.h>
#include <FreeRTOS.h>
#include <freertos/queue.h>

namespace robot::tasks {
    void battery_watch_task(void* parameter) {
        (void) parameter;

        Serial.println("[battery_watch_task]: Task started");

        constexpr TickType_t blink_half_period_ticks = pdMS_TO_TICKS(162.5);
        bool blink_phase_on = false;

        auto push_led_state = [](uint8_t pin, bool level) {
            IOExpanderCommand cmd{.pin = pin, .level = level};
            xQueueSend(robot::queues::io_command_queue, &cmd, 0);
        };

        while (true) {
            blink_phase_on = !blink_phase_on;
            robot::battery::BatteryStatus status = robot::battery::getStatus();

            const bool cell_1_critical = (status.cell_1_percentage <= robot::config::critical_batt_th);
            const bool cell_2_critical = (status.cell_2_percentage <= robot::config::critical_batt_th);
            const bool cell_3_critical = (status.cell_3_percentage <= robot::config::critical_batt_th);
            const bool cell_4_critical = (status.cell_4_percentage <= robot::config::critical_batt_th);

            const bool cell_1_warning = (status.cell_1_percentage <= robot::config::warning_batt_th);
            const bool cell_2_warning = (status.cell_2_percentage <= robot::config::warning_batt_th);
            const bool cell_3_warning = (status.cell_3_percentage <= robot::config::warning_batt_th);
            const bool cell_4_warning = (status.cell_4_percentage <= robot::config::warning_batt_th);

            // // print all cell voltages and percentages for debugging
            // Serial.printf("[battery_watch_task] cell1: %u%% (%u mV) %s%s\n",
            //               static_cast<unsigned int>(status.cell_1_percentage),
            //               static_cast<unsigned int>(status.cell_1_voltage_mv),
            //               cell_1_critical ? "CRITICAL " : "",
            //               cell_1_warning ? "WARNING" : "");
            //     Serial.printf("[battery_watch_task] cell2: %u%% (%u mV) %s%s\n",
            //               static_cast<unsigned int>(status.cell_2_percentage),
            //               static_cast<unsigned int>(status.cell_2_voltage_mv),
            //               cell_2_critical ? "CRITICAL " : "",
            //               cell_2_warning ? "WARNING" : "");
            //     Serial.printf("[battery_watch_task] cell3: %u%% (%u mV) %s%s\n",
            //                 static_cast<unsigned int>(status.cell_3_percentage),
            //                 static_cast<unsigned int>(status.cell_3_voltage_mv),
            //                 cell_3_critical ? "CRITICAL " : "",
            //                 cell_3_warning ? "WARNING" : "");
            //     Serial.printf("[battery_watch_task] cell4: %u%% (%u mV) %s%s\n",
            //                 static_cast<unsigned int>(status.cell_4_percentage),
            //                 static_cast<unsigned int>(status.cell_4_voltage_mv),
            //                 cell_4_critical ? "CRITICAL " : "",
            //                 cell_4_warning ? "WARNING" : "");



            push_led_state(
                robot::config::led_1_tca_pin,
                cell_1_critical ? blink_phase_on : cell_1_warning
            );

            push_led_state(
                robot::config::led_2_tca_pin,
                cell_2_critical ? blink_phase_on : cell_2_warning
            );

            push_led_state(
                robot::config::led_3_tca_pin,
                cell_3_critical ? blink_phase_on : cell_3_warning
            );

            push_led_state(
                robot::config::led_4_tca_pin,
                cell_4_critical ? blink_phase_on : cell_4_warning
            );

            vTaskDelay(blink_half_period_ticks);
        }
    }
}