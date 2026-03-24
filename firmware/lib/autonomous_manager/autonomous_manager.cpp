#include "autonomous_manager.h"

namespace robot::autonomous_manager {

void AutonomousManager::start(uint8_t sequenceId, uint32_t nowMs) {
  sequenceId_ = sequenceId;
  startMs_ = nowMs;
  active_ = true;
}

void AutonomousManager::stop() {
  active_ = false;
}

bool AutonomousManager::active() const {
  return active_;
}

JoystickCommand AutonomousManager::update(uint32_t nowMs, const JoystickCommand& fallback) const {
  if (!active_) {
    return fallback;
  }

  JoystickCommand out = fallback;
  const uint32_t elapsed = nowMs - startMs_;

  // Sequence 0: forward -> scan -> store.
  if (sequenceId_ == 0) {
    if (elapsed < 1500) {
      out.vx = 500;
      out.vy = 0;
      out.omega = 0;
      out.leftIntent = ActionIntent::NONE;
      out.rightIntent = ActionIntent::NONE;
    } else if (elapsed < 2500) {
      out.vx = 0;
      out.leftIntent = ActionIntent::GRAB;
      out.rightIntent = ActionIntent::GRAB;
    } else {
      out.vx = 0;
      out.leftIntent = ActionIntent::STORE;
      out.rightIntent = ActionIntent::STORE;
    }
  }

  return out;
}

}  // namespace robot::autonomous_manager
