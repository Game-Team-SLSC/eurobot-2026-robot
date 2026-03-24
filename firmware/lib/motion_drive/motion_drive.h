#pragma once

#include <FastAccelStepper.h>

#include "types.h"

namespace robot::motion_drive {

class MotionDrive {
 public:
  bool begin();
  void applyTarget(const MotionTarget& target);
  void stop();

 private:
    static constexpr uint8_t WHEEL_COUNT = 4;
    FastAccelStepperEngine engine_;
    FastAccelStepper* steppers_[WHEEL_COUNT] = {nullptr, nullptr, nullptr, nullptr};
  bool initialized_ = false;
};

}  // namespace robot::motion_drive
