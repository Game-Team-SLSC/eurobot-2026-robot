#include <pwm-controller.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h>
#include <buses.h>

namespace {
    Adafruit_PWMServoDriver leftPWM(robot::config::pca9685_left_i2c_config.address, *robot::buses::get(robot::config::pca9685_left_i2c_config.busId));
    Adafruit_PWMServoDriver rightPWM(robot::config::pca9685_right_i2c_config.address, *robot::buses::get(robot::config::pca9685_right_i2c_config.busId));
    Adafruit_PWMServoDriver miscPWM(robot::config::pca9685_misc_i2c_config.address, *(robot::buses::get(robot::config::pca9685_misc_i2c_config.busId)));
}

bool applyControllerSettings(Adafruit_PWMServoDriver& ctrl) {
    bool result = ctrl.begin();
    ctrl.setPWMFreq(50); // 50Hz for servos

    return result;
}

namespace robot::pwmcontroller {
    bool begin() {
        bool result = applyControllerSettings(leftPWM) &&
               applyControllerSettings(rightPWM) &&
               applyControllerSettings(miscPWM);

        if (result) {
            Serial.println("[pwmcontroller] PWM controllers initialized");
        } else {
            Serial.println("[pwmcontroller] Failed to initialize PWM controllers");
        }
        return result;
    }

    bool apply(const PWMCommand& command) {
        Adafruit_PWMServoDriver* pwm;
        switch (command.controller) {
            case robot::config::PWMController::LEFT:
                pwm = &leftPWM;
                break;
            case robot::config::PWMController::RIGHT:
                pwm = &rightPWM;
                break;
            case robot::config::PWMController::MISC:
                pwm = &miscPWM;
                break;
            default:
                return false;
        }
        
        
        Serial.printf("[pwmcontroller] Applying PWM command: value=%u\n", map(command.value, 0, 180, 115, 545));
        return pwm->setPWM(command.pin, 0, map(command.value, 0, 180, 115, 545));
    }

    bool apply(const CommandBatch<PWMCommand>& batch) {
        bool success = true;
        for (uint8_t i = 0; i < batch.count; ++i) {
            if (!robot::pwmcontroller::apply(batch.commands[i])) {
                success = false;
            }
        }
        return success;
    }
}