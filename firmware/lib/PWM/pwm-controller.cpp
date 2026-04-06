#include <pwm-controller.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h>
#include <buses.h>
#include <Logger.h>

namespace {
    Adafruit_PWMServoDriver leftPWM(robot::config::pca9685_left_i2c_config.address, *robot::buses::get(robot::config::pca9685_left_i2c_config.busId));
    Adafruit_PWMServoDriver rightPWM(robot::config::pca9685_right_i2c_config.address, *robot::buses::get(robot::config::pca9685_right_i2c_config.busId));
    Adafruit_PWMServoDriver miscPWM(robot::config::pca9685_misc_i2c_config.address, *(robot::buses::get(robot::config::pca9685_misc_i2c_config.busId)));

    bool leftReady = false;
    bool rightReady = false;
    bool miscReady = false;
}

bool applyControllerSettings(Adafruit_PWMServoDriver& ctrl) {
    bool result = ctrl.begin();
    ctrl.setPWMFreq(50); // 50Hz for servos

    return result;
}

namespace robot::pwmcontroller {
    bool begin() {
        leftReady = applyControllerSettings(leftPWM);
        rightReady = applyControllerSettings(rightPWM);
        miscReady = applyControllerSettings(miscPWM);

        const bool result = leftReady && rightReady && miscReady;

        if (result) {
            info("pwmcontroller", "Initialized");
        } else {
            error("pwmcontroller", "Failed to initialize PWM controllers (LEFT=%u RIGHT=%u MISC=%u)",
                  static_cast<unsigned int>(leftReady),
                  static_cast<unsigned int>(rightReady),
                  static_cast<unsigned int>(miscReady));
        }
        
        return result;
    }

    bool apply(const PWMCommand& command) {
        Adafruit_PWMServoDriver* pwm = nullptr;
        bool controllerReady = false;
        switch (command.controller) {
            case robot::config::PWMController::LEFT:
                pwm = &leftPWM;
                controllerReady = leftReady;
                break;
            case robot::config::PWMController::RIGHT:
                pwm = &rightPWM;
                controllerReady = rightReady;
                break;
            case robot::config::PWMController::MISC:
                pwm = &miscPWM;
                controllerReady = miscReady;
                break;
            default:
                error("pwmcontroller", "Invalid PWM controller id: %u", static_cast<unsigned int>(command.controller));
                return false;
        }

        if (!controllerReady) {
            warn("pwmcontroller", "Ignoring PWM command on non-initialized controller %u (pin=%u value=%u)",
                 static_cast<unsigned int>(command.controller),
                 static_cast<unsigned int>(command.pin),
                 static_cast<unsigned int>(command.value));
            return false;
        }
        
        
        bool ok = pwm->setPWM(command.pin, 0, map(command.value, 0, 180, 115, 545));
        return ok;
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