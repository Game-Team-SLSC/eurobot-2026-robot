#include "motion_drive.h"

#include <Arduino.h>

#include "config.h"
#include "hardware_pins.h"

namespace robot::motion_drive {

namespace {

struct WheelPins {
  int8_t step;
  int8_t dir;
};

constexpr WheelPins WHEELS[4] = {
    {pins::stepper::FL_STEP, pins::stepper::FL_DIR},
    {pins::stepper::FR_STEP, pins::stepper::FR_DIR},
    {pins::stepper::RL_STEP, pins::stepper::RL_DIR},
    {pins::stepper::RR_STEP, pins::stepper::RR_DIR},
};

int16_t ClampStepFreq(int32_t hz) {
  if (hz == 0) return 0;
  if (hz > 0) {
    if (hz < config::STEPPER_MIN_STEP_HZ) hz = config::STEPPER_MIN_STEP_HZ;
  } else {
    if (-hz < config::STEPPER_MIN_STEP_HZ) hz = -static_cast<int32_t>(config::STEPPER_MIN_STEP_HZ);
  }
  if (hz > static_cast<int32_t>(config::STEPPER_MAX_STEP_HZ)) hz = config::STEPPER_MAX_STEP_HZ;
  if (hz < -static_cast<int32_t>(config::STEPPER_MAX_STEP_HZ)) hz = -static_cast<int32_t>(config::STEPPER_MAX_STEP_HZ);
  return static_cast<int16_t>(hz);
}

void ApplyWheel(FastAccelStepper* stepper, int16_t stepHz) {
  if (stepper == nullptr) {
    return;
  }

  if (stepHz == 0) {
    stepper->stopMove();
    return;
  }

  stepper->setSpeedInHz(static_cast<uint32_t>(abs(stepHz)));
  if (stepHz > 0) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }
}

}  // namespace

bool MotionDrive::begin() {
  engine_.init();

  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const auto& wheel = WHEELS[i];
    if (wheel.step < 0 || wheel.dir < 0) {
      continue;
    }

    steppers_[i] = engine_.stepperConnectToPin(static_cast<uint8_t>(wheel.step));
    if (steppers_[i] == nullptr) {
      continue;
    }

    steppers_[i]->setDirectionPin(static_cast<uint8_t>(wheel.dir), false);
    steppers_[i]->setAutoEnable(false);
    steppers_[i]->setSpeedInHz(config::STEPPER_MIN_STEP_HZ);
    steppers_[i]->setAcceleration(10000);
    steppers_[i]->stopMove();
  }

  initialized_ = true;
  return true;
}

void MotionDrive::applyTarget(const MotionTarget& target) {
  if (!initialized_) {
    return;
  }

  // Simple mecanum-style mix in command units.
  const int32_t fl = target.vx + target.vy + target.omega;
  const int32_t fr = target.vx - target.vy - target.omega;
  const int32_t rl = target.vx - target.vy + target.omega;
  const int32_t rr = target.vx + target.vy - target.omega;

  constexpr int32_t DEN = config::DRIVE_MAX_LINEAR + config::DRIVE_MAX_LINEAR + config::DRIVE_MAX_ANGULAR;
  auto toStepHz = [](int32_t mixed) {
    const int32_t hz = (mixed * static_cast<int32_t>(config::STEPPER_MAX_STEP_HZ)) / DEN;
    return ClampStepFreq(hz);
  };

  ApplyWheel(steppers_[0], toStepHz(fl));
  ApplyWheel(steppers_[1], toStepHz(fr));
  ApplyWheel(steppers_[2], toStepHz(rl));
  ApplyWheel(steppers_[3], toStepHz(rr));
}

void MotionDrive::stop() {
  if (!initialized_) {
    return;
  }
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    if (steppers_[i] != nullptr) {
      steppers_[i]->stopMove();
    }
  }
}

}  // namespace robot::motion_drive
