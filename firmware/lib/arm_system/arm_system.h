#pragma once

#include "types.h"

namespace robot::arm_system {

class ArmSystem {
 public:
  bool begin();
  void applyCommand(const JoystickCommand& command);
  void update(uint32_t nowMs);
  ColorSensorsSnapshot pollColors();
  SideActuatorState leftState() const;
  SideActuatorState rightState() const;

 private:
  bool shouldFlipForSide(Side side, const JoystickCommand& command) const;
  void applyAuxOutputs(const JoystickCommand& command);
  void planArmPath(Side side, ArmPose target);
  void executeImmediatePose(Side side, ArmPose pose);

  ColorSensorsSnapshot lastColors_;
  SideActuatorState leftState_;
  SideActuatorState rightState_;
  ArmPose pendingLeftPose_ = ArmPose::IDLE;
  ArmPose pendingRightPose_ = ArmPose::IDLE;
  uint32_t pendingLeftAtMs_ = 0;
  uint32_t pendingRightAtMs_ = 0;
  bool initialized_ = false;
  uint16_t pumpMask_ = 0;
};

}  // namespace robot::arm_system
