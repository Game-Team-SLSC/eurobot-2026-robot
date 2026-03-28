#include "state.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <logging.h>

namespace {
SemaphoreHandle_t g_stateMutex = nullptr;
robot::state::GlobalState g_state{};
bool g_initialized = false;
} // namespace

namespace robot::state {
    bool begin() {
        if (g_initialized) {
            robot::logging::info("state", "begin called while already initialized");
            return true;
        }

        g_stateMutex = xSemaphoreCreateMutex();
        if (g_stateMutex == nullptr) {
            robot::logging::warn("state", "mutex allocation failed");
            return false;
        }

        g_state = GlobalState{};
        g_initialized = true;
        robot::logging::info("state", "state store initialized");
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

        static bool lastConnectedLogged = false;
        static bool hasLoggedState = false;
        if (!hasLoggedState || (lastConnectedLogged != connected)) {
            hasLoggedState = true;
            lastConnectedLogged = connected;
            if (connected) {
                robot::logging::info("state", "radio connected=true");
            } else {
                robot::logging::warn("state", "radio connected=false");
            }
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

        static bool lastTimedOutLogged = true;
        static bool hasLoggedTimeout = false;
        if (!hasLoggedTimeout || (lastTimedOutLogged != timedOut)) {
            hasLoggedTimeout = true;
            lastTimedOutLogged = timedOut;
            if (timedOut) {
                robot::logging::warn("state", "radio timeout=true");
            } else {
                robot::logging::info("state", "radio timeout=false");
            }
        }

        xSemaphoreGive(g_stateMutex);
        return true;
    }
}
