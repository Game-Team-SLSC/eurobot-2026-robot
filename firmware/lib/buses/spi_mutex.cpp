#include "spi_mutex.h"

#include <freertos/semphr.h>

namespace {
SemaphoreHandle_t g_spiMutex = nullptr;
} // namespace

namespace robot::spi_mutex {

void begin() {
    if (g_spiMutex == nullptr) {
        g_spiMutex = xSemaphoreCreateMutex();
    }
}

bool lock(TickType_t timeoutTicks) {
    if (g_spiMutex == nullptr) {
        return false;
    }
    return xSemaphoreTake(g_spiMutex, timeoutTicks) == pdTRUE;
}

void unlock() {
    if (g_spiMutex != nullptr) {
        xSemaphoreGive(g_spiMutex);
    }
}

Guard::Guard(TickType_t timeoutTicks): locked(lock(timeoutTicks)) {}

Guard::~Guard() {
    if (locked) {
        unlock();
    }
}

bool Guard::isLocked() const {
    return locked;
}

} // namespace robot::spi_mutex
