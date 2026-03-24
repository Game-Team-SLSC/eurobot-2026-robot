#pragma once

#include "types.h"

namespace robot {

class SystemState {
 public:
  bool begin();

  SystemSnapshot getSnapshot() const;

  void setMode(RobotMode mode);
  void setRfLinkAlive(bool alive);
  void setLastRfRxMs(uint32_t timestampMs);
  void setBattery(const BatteryStatus& battery);
  void setColors(const ColorSensorsSnapshot& colors);
  void setMotionTarget(const MotionTarget& target);
  void setLastCommand(const JoystickCommand& command);
  void setLastAck(const CommandAck& ack);
  void setLeftActuator(const SideActuatorState& state);
  void setRightActuator(const SideActuatorState& state);
  void setStorage(const StorageState& state);

 private:
  mutable void* mutex_ = nullptr;
  SystemSnapshot snapshot_;
};

}  // namespace robot
