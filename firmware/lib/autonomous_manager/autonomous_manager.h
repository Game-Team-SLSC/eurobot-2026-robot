#pragma once

#include "types.h"

namespace robot::autonomous_manager {

class AutonomousManager {
 public:
  void start(uint8_t sequenceId, uint32_t nowMs);
  void stop();
  bool active() const;
  JoystickCommand update(uint32_t nowMs, const JoystickCommand& fallback) const;

 private:
  bool active_ = false;
  uint8_t sequenceId_ = 0;
  uint32_t startMs_ = 0;
};

}  // namespace robot::autonomous_manager
