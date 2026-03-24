#pragma once

#include <cstdint>

#include "types.h"

namespace robot {

enum class EventType : uint8_t {
  NONE = 0,
  RF24_COMMAND = 1,
  BATTERY_UPDATE = 2,
  MOTION_UPDATE = 3,
  SAFETY_ALERT = 4,
};

struct Event {
  EventType type = EventType::NONE;
  uint32_t timestampMs = 0;
  JoystickCommand command;
  BatteryStatus battery;
};

class EventBus {
 public:
  bool begin(uint32_t queueLength);
  bool publish(const Event& event, uint32_t timeoutMs = 0);
  bool wait(Event* outEvent, uint32_t timeoutMs);

 private:
  void* queue_ = nullptr;
};

}  // namespace robot
