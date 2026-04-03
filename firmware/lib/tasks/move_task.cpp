#include <Arduino.h>
#include <tasks.h>
#include <movers.h>
#include <queues.h>
#include <state.h>
#include <commands.h>
#include <config.h>

namespace {
constexpr int16_t JOYSTICK_CENTER = 128;
constexpr int16_t AXIS_DEADZONE = 4;
constexpr int16_t ROTATE_DEADZONE = 6;

inline int8_t normalizeAxisValue(int16_t value, int16_t deadzone) {
    if ((value <= deadzone) && (value >= -deadzone)) {
        return 0;
    }
    return static_cast<int8_t>(value);
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

}

namespace robot::tasks {
    void move_task(void* parameter) {
        (void) parameter;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xPeriode = pdMS_TO_TICKS(5);

        Serial.println("[control] task started");

        while (true) {
            vTaskDelayUntil(&xLastWakeTime, xPeriode);

            if (robot::state::get().action == robot::config::Action::IDLE) {
                GlobalState state = robot::state::get();

                MotionCommand cmd{};
            
                Vec3 currentVel = robot::movers::getCurrentVelocity();
            
                int16_t forward = static_cast<int16_t>(state.remoteData.joystickLeft.y) - JOYSTICK_CENTER;
                const int16_t forwardClamped = (forward > 127) ? 127 : (forward < -127 ? -127 : forward);
                cmd.forward = applyExpoResponse(normalizeAxisValue(forwardClamped, AXIS_DEADZONE));
                
                int16_t strafe = static_cast<int16_t>(state.remoteData.joystickLeft.x) - JOYSTICK_CENTER;
                const int16_t strafeClamped = (strafe > 127) ? 127 : (strafe < -127 ? -127 : strafe);
                cmd.strafe = applyExpoResponse(normalizeAxisValue(strafeClamped, AXIS_DEADZONE));
        
                int16_t rotate = static_cast<int16_t>(state.remoteData.joystickRight.x) - JOYSTICK_CENTER;
                const int16_t rotateClamped = (rotate > 127) ? 127 : (rotate < -127 ? -127 : rotate);
                cmd.rotate = applyExpoResponse(normalizeAxisValue(rotateClamped, ROTATE_DEADZONE));

                const float lowSpeedScale = state.lowSpeedMode ? 0.1f : 1.0f;
                cmd.forward *= lowSpeedScale;
                cmd.strafe *= lowSpeedScale;
                cmd.rotate *= lowSpeedScale;
        
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

                robot::movers::drive(cmd);   
            } else {
                MotionCommand cmd;
                if (xQueueReceive(robot::queues::motion_command_queue, &cmd, 0) == pdPASS) {
                    robot::movers::goToTarget(cmd);
                }
            }
        }
    }
}