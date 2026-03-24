#pragma once

#include "event_bus.h"
#include "system_state.h"

namespace robot {

struct TaskContext {
  EventBus* bus = nullptr;
  SystemState* state = nullptr;
};

}  // namespace robot
