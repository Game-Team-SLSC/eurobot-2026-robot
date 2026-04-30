#include <tasks.h>
#include <Arduino.h>
#include <battery.h>
#include <config.h>
#include <queues.h>
#include <FreeRTOS.h>
#include <freertos/queue.h>
#include <Logger.h>
#include <state.h>
#include <movers.h>

namespace robot::tasks {
    void battery_watch_task(void* parameter) {
        (void) parameter;

        info("battery_watch_task", "Task started");

        constexpr TickType_t blink_half_period_ticks = pdMS_TO_TICKS(162.5);
        bool blink_phase_on = false;
        bool last_critical_state = false;
        bool last_warning_state = false;
        bool last_unplugged_state = false;

        auto push_led_state = [](uint8_t pin, bool level) {
            IOExpanderCommand cmd{.pin = pin, .level = level};
            xQueueSend(robot::queues::io_command_queue, &cmd, 0);
        };

        while (true) {
            GlobalState state = robot::state::get();

            Vec3 current_velocity = robot::movers::getCurrentVelocity();
            uint8_t speed = static_cast<uint8_t>(sqrt(pow(current_velocity.forward, 2) + pow(current_velocity.strafe, 2)));

            if (speed > 0) continue;
            
            blink_phase_on = !blink_phase_on;
            robot::battery::BatteryStatus status = robot::battery::getStatus();

            robot::state::setBatteryStatus(status);

            const bool cell_1_critical = (status.cell_1_percentage <= robot::config::critical_batt_th);
            const bool cell_2_critical = (status.cell_2_percentage <= robot::config::critical_batt_th);
            const bool cell_3_critical = (status.cell_3_percentage <= robot::config::critical_batt_th);
            const bool cell_4_critical = (status.cell_4_percentage <= robot::config::critical_batt_th);

            const bool cell_1_warning = (status.cell_1_percentage <= robot::config::warning_batt_th);
            const bool cell_2_warning = (status.cell_2_percentage <= robot::config::warning_batt_th);
            const bool cell_3_warning = (status.cell_3_percentage <= robot::config::warning_batt_th);
            const bool cell_4_warning = (status.cell_4_percentage <= robot::config::warning_batt_th);

            bool is_critical = (status.cell_1_percentage <= 0 || status.cell_2_percentage <= 0 || status.cell_3_percentage <= 0 || status.cell_4_percentage <= 0) ||
                               (cell_1_critical || cell_2_critical || cell_3_critical || cell_4_critical);
            bool is_warning = (cell_1_warning || cell_2_warning || cell_3_warning || cell_4_warning);
            bool is_unplugged = (status.voltage_mv < 1000);

            // Log only once when state transitions
            if (is_unplugged && !last_unplugged_state) {
                warn("battery_watch_task", "Battery unplugged! Total voltage below 1000mV.");
                last_unplugged_state = true;
                last_critical_state = false;
                last_warning_state = false;
                state::setCriticalBattery(true);
            } else if (!is_unplugged && last_unplugged_state) {
                last_unplugged_state = false;
            } else if (is_critical && !last_critical_state && !is_unplugged) {
                warn("battery_watch_task", "Critical battery level detected! Stopping the robot.");
                last_critical_state = true;
                last_warning_state = false;
                state::setCriticalBattery(true);
            } else if (is_warning && !last_warning_state && !is_critical) {
                warn("battery_watch_task", "Battery level warning.");
                last_warning_state = true;
                last_critical_state = false;
                state::setCriticalBattery(false);
            } else if (!is_critical && !is_warning) {
                last_critical_state = false;
                last_warning_state = false;
                state::setCriticalBattery(false);
            } else if (is_critical) {
                state::setCriticalBattery(true);
            } else if (is_warning) {
                state::setCriticalBattery(false);
            }

            if (is_unplugged) {
                state::setCriticalBattery(true);
            }

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