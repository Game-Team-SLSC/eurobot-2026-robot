#include <Arduino.h>

#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <printf.h>
#include <remote.h>
#include <ioexpander.h>
#include <pwm-controller.h>
#include <buses.h>
#include <movers.h>

QueueHandle_t g_ioCommandQueue = nullptr;
QueueHandle_t g_motionCommandMailbox = nullptr;
QueueHandle_t g_pwmCommand_Mailbox = nullptr;
QueueHandle_t g_actionCommandQueue = nullptr;

enum class Servo: uint8_t {
        BACK_LEFT_TURNER,
        FRONT_LEFT_TURNER,
        BACK_RIGHT_TURNER,
        FRONT_RIGHT_TURNER,
        BACK_LEFT_GRABBER,
        FRONT_LEFT_GRABBER,
        BACK_RIGHT_GRABBER,
        FRONT_RIGHT_GRABBER,
    };

struct MotionCommand {
    int8_t forward = 0;
    int8_t strafe = 0;
    int8_t rotate = 0;
    uint32_t timestampMs = 0;
};

inline int8_t normalizeAxisValue(int16_t value) {
    if (value <= 1 && value >= -1) {
        return 0;
    }
    return value;
}

inline int8_t applyExpoResponse(int8_t value) {
    constexpr float AXIS_MAX = 127.0f;
    constexpr float EXPO_GAIN = 3.0f;

    if (value == 0) {
        return 0;
    }

    const float sign = (value < 0) ? -1.0f : 1.0f;
    const float x = fabsf(static_cast<float>(value)) / AXIS_MAX;
    const float y = (expf(EXPO_GAIN * x) - 1.0f) / (expf(EXPO_GAIN) - 1.0f);

    int16_t shaped = static_cast<int16_t>(lroundf(sign * y * AXIS_MAX));
    if (shaped > 127) {
        shaped = 127;
    }
    if (shaped < -127) {
        shaped = -127;
    }
    return static_cast<int8_t>(shaped);
}

inline bool hasOppositeSign(int16_t command, int16_t velocity) {
    if ((command == 0) || (velocity == 0)) {
        return false;
    }
    return ((command > 0) && (velocity < 0)) || ((command < 0) && (velocity > 0));
}

inline int16_t velocitySignDeadzone(int16_t value) {
    constexpr int16_t VELOCITY_SIGN_EPSILON = 3;
    if ((value >= -VELOCITY_SIGN_EPSILON) && (value <= VELOCITY_SIGN_EPSILON)) {
        return 0;
    }
    return value;
}

void commTask(void* parameter) {
    (void) parameter;

    Serial.println("[comm] task started");

    while (true) {
        robot::types::RemoteData data;
        if (robot::remote::fetch(data)) {
            Serial.println("[comm] Frame received");

            // Motion

            MotionCommand cmd{};

            robot::movers::Vec3 currentVel = robot::movers::getCurrentVelocity();

            int16_t forward = static_cast<int16_t>(data.joystickLeft.y) - 127;
            const int16_t forwardClamped = (forward > 127) ? 127 : (forward < -127 ? -127 : forward);
            cmd.forward = applyExpoResponse(normalizeAxisValue(forwardClamped));
            
            int16_t strafe = static_cast<int16_t>(data.joystickLeft.x) - 127;
            const int16_t strafeClamped = (strafe > 127) ? 127 : (strafe < -127 ? -127 : strafe);
            cmd.strafe = applyExpoResponse(normalizeAxisValue(strafeClamped));

            int16_t rotate = static_cast<int16_t>(data.joystickRight.x) - 127;
            const int16_t rotateClamped = (rotate > 127) ? 127 : (rotate < -127 ? -127 : rotate);
            cmd.rotate = applyExpoResponse(normalizeAxisValue(rotateClamped));

            const int16_t velForward = velocitySignDeadzone(currentVel.forward);
            const int16_t velStrafe = velocitySignDeadzone(currentVel.strafe);
            const int16_t velRotate = velocitySignDeadzone(currentVel.rotate);

            if (hasOppositeSign(cmd.forward, velForward)) {
                cmd.forward = 0;
            }
            if (hasOppositeSign(cmd.strafe, velStrafe)) {
                cmd.strafe = 0;
            }
            if (hasOppositeSign(cmd.rotate, velRotate)) {
                cmd.rotate = 0;
            }

            cmd.timestampMs = millis();

            xQueueSend(g_motionCommandMailbox, &cmd, 0);

            // Actions

            for (uint8_t btnIdx = 0; btnIdx < static_cast<uint8_t>(robot::config::Button::_BUTTON_COUNT); ++btnIdx) {
                if (!data.buttons[btnIdx]) {
                    continue;
                }
                if (btnIdx == static_cast<uint8_t>(robot::config::turn_action_btn)) {
                    xQueueSend(g_actionCommandQueue, static_cast<uint8_t>(robot::config::Actions::TURN), 0);
                }
            }

        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void controlTask(void* parameter) {
    (void) parameter;

    Serial.println("[control] task started");

    while (true) {
        MotionCommand incoming;
        xQueueReceive(g_motionCommandMailbox, &incoming, 0);
        robot::movers::drive(incoming.forward, incoming.strafe, incoming.rotate);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ioTask(void* parameter) {
    (void) parameter;

    Serial.println("[io] task started");

    while (true) {
        robot::ioexpander::Command command{};
        if ((g_ioCommandQueue != nullptr) &&
            (xQueueReceive(g_ioCommandQueue, &command, pdMS_TO_TICKS(100)) == pdPASS)) {
            if (!robot::ioexpander::apply(command)) {
                Serial.println("[io] Failed to apply ioexpander command");
            }
        }
    }
}

void miscControl(void* parameter) {
    (void) parameter;

    Serial.println("[misc]: Task started");

    while (true) {

        robot::pwmcontroller::Command pwmCommand{};
        if ((g_pwmCommand_Mailbox != nullptr) &&
            (xQueueReceive(g_pwmCommand_Mailbox, &pwmCommand, pdMS_TO_TICKS(100)) == pdPASS)) {
            if (!robot::pwmcontroller::apply(pwmCommand)) {
                Serial.println("[misc] Failed to apply pwm command");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void setup() {
    Serial.begin(115200);
    printf_begin();
    delay(1000); // Allow time for Serial to initialize
    
    g_ioCommandQueue = xQueueCreate(16, sizeof(robot::ioexpander::Command));
    g_motionCommandMailbox = xQueueCreate(1, sizeof(MotionCommand));
    g_pwmCommand_Mailbox = xQueueCreate(1, sizeof(robot::pwmcontroller::Command));
    g_actionCommandQueue = xQueueCreate(16, sizeof(uint8_t));

    robot::buses::begin();
    robot::ioexpander::begin();
    robot::movers::begin();
    robot::pwmcontroller::begin();
    robot::remote::connect();

    xTaskCreatePinnedToCore(commTask, "commTask", 4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(ioTask, "ioTask", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(controlTask, "controlTask", 4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(miscControl, "miscControl", 4096, nullptr, 1, nullptr, 0);

    robot::ioexpander::Command cmd{};
    cmd.pin = 7;
    cmd.level = false;
    xQueueSend(g_ioCommandQueue, &cmd, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
