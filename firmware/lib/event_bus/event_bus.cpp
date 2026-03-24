#include "event_bus.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace robot {

bool EventBus::begin(uint32_t queueLength) {
  queue_ = static_cast<void*>(xQueueCreate(queueLength, sizeof(Event)));
  return queue_ != nullptr;
}

bool EventBus::publish(const Event& event, uint32_t timeoutMs) {
  if (queue_ == nullptr) {
    return false;
  }

  const TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
  return xQueueSend(static_cast<QueueHandle_t>(queue_), &event, ticks) == pdTRUE;
}

bool EventBus::wait(Event* outEvent, uint32_t timeoutMs) {
  if (queue_ == nullptr || outEvent == nullptr) {
    return false;
  }

  const TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
  return xQueueReceive(static_cast<QueueHandle_t>(queue_), outEvent, ticks) == pdTRUE;
}

}  // namespace robot
