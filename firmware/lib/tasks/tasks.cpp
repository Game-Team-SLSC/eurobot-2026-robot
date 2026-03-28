#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <config.h>
#include <RemoteData.h>
#include <buses.h>
#include <ioexpander.h>
#include <movers.h>
#include <remote.h>
#include <state.h>
#include <logging.h>

namespace {
QueueHandle_t g_motionCommandMailbox = nullptr;
QueueHandle_t g_ioCommandQueue = nullptr;
bool g_started = false;

constexpr uint32_t RADIO_RETRY_DELAY_MS = 1000;
constexpr uint32_t RADIO_TIMEOUT_MS = robot::config::rf_timeout_ms;

constexpr uint16_t COMM_TASK_STACK_WORDS = 4096;
constexpr uint16_t CONTROL_TASK_STACK_WORDS = 4096;
constexpr uint16_t IO_TASK_STACK_WORDS = 3072;
constexpr UBaseType_t COMM_TASK_PRIORITY = 2;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 3;
constexpr UBaseType_t IO_TASK_PRIORITY = 2;
constexpr uint16_t IO_COMMAND_QUEUE_LENGTH = 16;

int8_t normalizeAxis(uint8_t rawValue, bool invert) {
    int16_t centered = static_cast<int16_t>(rawValue) - 128;
    if (invert) {
        centered = -centered;
    }
    centered = constrain(centered, -127, 127);
    return static_cast<int8_t>(centered);
}

TickType_t commandPeriodTicks() {
    const uint16_t hz = (robot::config::rf_frequency == 0) ? 1 : robot::config::rf_frequency;
    const TickType_t ticks = pdMS_TO_TICKS(1000 / hz);
    return (ticks == 0) ? 1 : ticks;
}

TickType_t controlPeriodTicks() {
    const uint16_t hz = (robot::config::movers_control_hz == 0) ? 1 : robot::config::movers_control_hz;
    const TickType_t ticks = pdMS_TO_TICKS(1000 / hz);
    return (ticks == 0) ? 1 : ticks;
}
} // namespace

namespace robot::tasks {
    bool begin() {
        if (g_started) {
            robot::logging::info("tasks", "begin called while already started");
            return true;
        }

        robot::logging::info("tasks", "task system init start");

        if (!robot::buses::begin()) {
            robot::logging::warn("tasks", "buses init failed");
            return false;
        }

        if (!robot::ioexpander::begin()) {
            robot::logging::warn("tasks", "ioexpander init failed");
            return false;
        }

        if (!robot::movers::begin()) {
            robot::logging::warn("tasks", "movers init failed");
            return false;
        }

        g_motionCommandMailbox = xQueueCreate(1, sizeof(MotionCommand));
        if (g_motionCommandMailbox == nullptr) {
            robot::logging::warn("tasks", "motion mailbox allocation failed");
            return false;
        }

        g_ioCommandQueue = xQueueCreate(IO_COMMAND_QUEUE_LENGTH, sizeof(robot::ioexpander::Command));
        if (g_ioCommandQueue == nullptr) {
            robot::logging::warn("tasks", "io command queue allocation failed");
            return false;
        }

        if (!robot::state::begin()) {
            robot::logging::warn("tasks", "state init failed");
            return false;
        }

        if (xTaskCreate(commTask,
                        "commTask",
                        COMM_TASK_STACK_WORDS,
                        nullptr,
                        COMM_TASK_PRIORITY,
                        nullptr) != pdPASS) {
            robot::logging::warn("tasks", "comm task creation failed");
            return false;
        }

        // controlTask intentionally disabled while isolating RF24/comm path.

        if (xTaskCreate(ioTask,
                        "ioTask",
                        IO_TASK_STACK_WORDS,
                        nullptr,
                        IO_TASK_PRIORITY,
                        nullptr) != pdPASS) {
            robot::logging::warn("tasks", "io task creation failed");
            return false;
        }

        robot::ioexpander::Command bootMotorsEnable{};
        bootMotorsEnable.type = robot::ioexpander::CommandType::SetPin;
        bootMotorsEnable.pin = robot::config::tca_pin_motors_enable;
        bootMotorsEnable.level = true;
        submitIoCommand(bootMotorsEnable);

        g_started = true;
        robot::logging::info("tasks", "task system init complete");
        return true;
    }

    bool getGlobalState(GlobalState& out) {
        return robot::state::get(out);
    }

    bool submitIoCommand(const robot::ioexpander::Command& command) {
        if (g_ioCommandQueue == nullptr) {
            robot::logging::warn("tasks", "submitIoCommand called before queue init");
            return false;
        }

        const bool queued = xQueueSend(g_ioCommandQueue, &command, pdMS_TO_TICKS(5)) == pdTRUE;
        if (!queued) {
            robot::logging::warn("tasks", "submitIoCommand queue full or unavailable");
        }
        return queued;
    }

    void commTask(void* parameter) {
        (void) parameter;

        robot::logging::info("comm", "task started");

        while (!robot::remote::connect()) {
            robot::state::setRadioConnected(false);
            robot::logging::warn("comm", "RF24 connect failed, retrying in 1s");
            vTaskDelay(pdMS_TO_TICKS(RADIO_RETRY_DELAY_MS));
        }

        robot::state::setRadioConnected(true);
        robot::logging::info("comm", "RF24 connected");

        const TickType_t period = commandPeriodTicks();
        TickType_t lastWake = xTaskGetTickCount();
        uint32_t rxCount = 0;

        while (true) {
            robot::types::RemoteData frame;
            if (robot::remote::fetch(frame)) {
                MotionCommand command;
                robot::logging::infof("comm", "received frame: fwd=%d strafe=%d rot=%d",
                                      static_cast<int>(frame.joystickLeft.y),
                                      static_cast<int>(frame.joystickLeft.x),
                                      static_cast<int>(frame.joystickRight.x));
                command.forward = normalizeAxis(frame.joystickLeft.y, true);
                command.strafe = normalizeAxis(frame.joystickLeft.x, false);
                command.rotate = normalizeAxis(frame.joystickRight.x, false);
                command.timestampMs = millis();

                if (g_motionCommandMailbox != nullptr) {
                    xQueueOverwrite(g_motionCommandMailbox, &command);
                }

                robot::state::setFrameReceivedAt(command.timestampMs);
                ++rxCount;
                if ((rxCount % 50U) == 0U) {
                    robot::logging::infof("comm", "frames=%lu fwd=%d strafe=%d rot=%d",
                                          static_cast<unsigned long>(rxCount),
                                          static_cast<int>(command.forward),
                                          static_cast<int>(command.strafe),
                                          static_cast<int>(command.rotate));
                }
            }

            vTaskDelayUntil(&lastWake, period);
        }
    }

    void controlTask(void* parameter) {
        (void) parameter;

        robot::logging::info("control", "task started");

        const TickType_t period = controlPeriodTicks();
        TickType_t lastWake = xTaskGetTickCount();

        MotionCommand latestCommand{};
        bool lastTimedOut = true;

        while (true) {
            MotionCommand incoming{};
            if ((g_motionCommandMailbox != nullptr) && (xQueueReceive(g_motionCommandMailbox, &incoming, 0) == pdPASS)) {
                latestCommand = incoming;
            }

            const uint32_t nowMs = millis();
            const bool timedOut = (latestCommand.timestampMs == 0) ||
                                  ((nowMs - latestCommand.timestampMs) > RADIO_TIMEOUT_MS);

            if (timedOut) {
                robot::movers::drive(0, 0, 0);
            } else {
                robot::movers::drive(latestCommand.forward,
                                    latestCommand.strafe,
                                    latestCommand.rotate);
            }

            if (timedOut != lastTimedOut) {
                lastTimedOut = timedOut;
                if (timedOut) {
                    robot::logging::warn("control", "radio timeout -> stopping motors");
                } else {
                    robot::logging::info("control", "radio recovered -> control active");
                }
            }

            robot::state::setControlSnapshot(timedOut, latestCommand);

            vTaskDelayUntil(&lastWake, period);
        }
    }

    void ioTask(void* parameter) {
        (void) parameter;

        robot::logging::info("io", "task started");

        while (true) {
            robot::ioexpander::Command command{};
            if ((g_ioCommandQueue != nullptr) &&
                (xQueueReceive(g_ioCommandQueue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
                if (!robot::ioexpander::apply(command)) {
                    robot::logging::warn("io", "failed to apply ioexpander command");
                }
            }
        }
    }
}