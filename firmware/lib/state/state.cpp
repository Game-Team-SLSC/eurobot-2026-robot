#include "state.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace {
SemaphoreHandle_t g_stateMutex = nullptr;
robot::state::GlobalState g_state{};
bool g_initialized = false;
} // namespace

namespace robot::state {
    bool begin() {
        if (g_initialized) {
            return true;
        }

        g_stateMutex = xSemaphoreCreateMutex();
        if (g_stateMutex == nullptr) {
            return false;
        }

        g_state = GlobalState{};
        g_initialized = true;
        return true;
    }

    bool get(GlobalState& out) {
        if (g_stateMutex == nullptr) {
            return false;
        }

        if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return false;
        }

        out = g_state;
        xSemaphoreGive(g_stateMutex);
        return true;
    }

    bool setRadioConnected(bool connected) {
        if (g_stateMutex == nullptr) {
            return false;
        }

        if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return false;
        }

        g_state.radioConnected = connected;
        if (!connected) {
            g_state.radioTimedOut = true;
        }

        xSemaphoreGive(g_stateMutex);
        return true;
    }

    bool setFrameReceivedAt(uint32_t timestampMs) {
        if (g_stateMutex == nullptr) {
            return false;
        }

        if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return false;
        }

        g_state.lastFrameTimestampMs = timestampMs;
        g_state.radioTimedOut = false;

        xSemaphoreGive(g_stateMutex);
        return true;
    }

    bool setControlSnapshot(bool timedOut, const MotionCommand& command) {
        if (g_stateMutex == nullptr) {
            return false;
        }

        if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return false;
        }

        g_state.radioTimedOut = timedOut;
        if (timedOut) {
            g_state.activeCommand = MotionCommand{};
        } else {
            g_state.activeCommand = command;
        }

        xSemaphoreGive(g_stateMutex);
        return true;
    }
}
