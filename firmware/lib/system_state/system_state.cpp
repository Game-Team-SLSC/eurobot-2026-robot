#include "system_state.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace robot {

bool SystemState::begin() {
  mutex_ = static_cast<void*>(xSemaphoreCreateMutex());
  snapshot_.uptimeMs = millis();
  snapshot_.mode = RobotMode::BOOT;
  return mutex_ != nullptr;
}

SystemSnapshot SystemState::getSnapshot() const {
  SystemSnapshot copy;
  if (mutex_ == nullptr) {
    return copy;
  }

  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    copy = snapshot_;
    copy.uptimeMs = millis();
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
  return copy;
}

void SystemState::setMode(RobotMode mode) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.mode = mode;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setRfLinkAlive(bool alive) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.rfLinkAlive = alive;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setLastRfRxMs(uint32_t timestampMs) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.lastRfRxMs = timestampMs;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setBattery(const BatteryStatus& battery) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.battery = battery;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setColors(const ColorSensorsSnapshot& colors) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.colors = colors;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setMotionTarget(const MotionTarget& target) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.target = target;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setLastCommand(const JoystickCommand& command) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.lastCommand = command;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setLastAck(const CommandAck& ack) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.lastAck = ack;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setLeftActuator(const SideActuatorState& state) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.leftActuator = state;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setRightActuator(const SideActuatorState& state) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.rightActuator = state;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

void SystemState::setStorage(const StorageState& state) {
  if (mutex_ == nullptr) {
    return;
  }
  if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(mutex_), pdMS_TO_TICKS(5)) == pdTRUE) {
    snapshot_.storage = state;
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(mutex_));
  }
}

}  // namespace robot
