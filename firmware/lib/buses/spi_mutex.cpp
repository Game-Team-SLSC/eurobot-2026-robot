#include "spi_mutex.h"

#include <freertos/semphr.h>

namespace {
SemaphoreHandle_t g_spiMutex = nullptr;
portMUX_TYPE g_spiMutexInitMux = portMUX_INITIALIZER_UNLOCKED;

void ensureInitialized() {
    if (g_spiMutex != nullptr) {
        return;
    }

    taskENTER_CRITICAL(&g_spiMutexInitMux);
    if (g_spiMutex == nullptr) {
        g_spiMutex = xSemaphoreCreateMutex();
    }
    taskEXIT_CRITICAL(&g_spiMutexInitMux);
}
} // namespace

namespace robot::spi_mutex {

void begin() {
    ensureInitialized();
}

bool lock(TickType_t timeoutTicks) {
    ensureInitialized();
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
