#include <pwm-controller.h>
#include <PCA9685.h>
#include <config.h>
#include <buses.h>

namespace {
    PCA9685 leftPWM(robot::config::pca9685_left_i2c_config.address, robot::buses::get(robot::config::pca9685_left_i2c_config.busId));
    PCA9685 rightPWM(robot::config::pca9685_right_i2c_config.address, robot::buses::get(robot::config::pca9685_right_i2c_config.busId));
    PCA9685 miscPWM(robot::config::pca9685_misc_i2c_config.address, robot::buses::get(robot::config::pca9685_misc_i2c_config.busId));
}

bool applyControllerSettings(PCA9685& ctrl) {
    ctrl.begin();
    ctrl.setFrequency(50); // 50Hz for servos
    return true;
}

namespace pwmcontroller {
    bool begin() {
        return applyControllerSettings(leftPWM) &&
               applyControllerSettings(rightPWM) &&
               applyControllerSettings(miscPWM);
    }

    bool apply(const robot::pwmcontroller::Command& command) {
        PCA9685* pwm;
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
        return pwm->setPWM(command.pin, command.value) == PCA9685_OK;
    }
}