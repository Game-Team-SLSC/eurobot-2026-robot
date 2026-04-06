#include <Arduino.h>
#include <tasks.h>
#include <movers.h>
#include <queues.h>
#include <state.h>
#include <commands.h>
#include <config.h>

namespace {
constexpr int16_t JOYSTICK_CENTER = 127;
constexpr int16_t JOYSTICK_CENTER_SNAP = 2;
constexpr int16_t AXIS_DEADZONE = 4;
constexpr int16_t ROTATE_DEADZONE = 8;
constexpr float STRAIGHT_FORWARD_MIN = 20.0f;
constexpr float STRAIGHT_STRAFE_MAX = 10.0f;
constexpr float ROTATE_DRIFT_MAX = 12.0f;
constexpr uint32_t MOVE_DEBUG_PERIOD_MS = 120;

inline int16_t centeredAxisFromRaw(uint8_t rawValue) {
    int16_t centered = static_cast<int16_t>(rawValue) - JOYSTICK_CENTER;
    if ((centered <= JOYSTICK_CENTER_SNAP) && (centered >= -JOYSTICK_CENTER_SNAP)) {
        return 0;
    }
    if (centered > 127) {
        return 127;
    }
    if (centered < -127) {
        return -127;
    }
    return centered;
}

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
            
                const int16_t forward = centeredAxisFromRaw(state.remoteData.joystickLeft.y);
                cmd.forward = applyExpoResponse(normalizeAxisValue(forward, AXIS_DEADZONE));
                
                const int16_t strafe = centeredAxisFromRaw(state.remoteData.joystickLeft.x);
                cmd.strafe = applyExpoResponse(normalizeAxisValue(strafe, AXIS_DEADZONE));
        
                const int16_t rotate = centeredAxisFromRaw(state.remoteData.joystickRight.x);
                cmd.rotate = applyExpoResponse(normalizeAxisValue(rotate, ROTATE_DEADZONE));

                const float lowSpeedScale = state.lowSpeedMode ? 0.1f : 1.0f;
                cmd.forward *= lowSpeedScale;
                cmd.strafe *= lowSpeedScale;
                cmd.rotate *= lowSpeedScale;

                bool rotateDriftClamped = false;
                if ((fabsf(cmd.forward) >= STRAIGHT_FORWARD_MIN) &&
                    (fabsf(cmd.strafe) <= STRAIGHT_STRAFE_MAX) &&
                    (fabsf(cmd.rotate) <= ROTATE_DRIFT_MAX)) {
                    rotateDriftClamped = true;
                    cmd.rotate = 0;
                }
        
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