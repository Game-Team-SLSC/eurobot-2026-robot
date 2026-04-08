#include <state.h>
#include <Arduino.h>

GlobalState globalState{};

namespace robot::state {
    SemaphoreHandle_t mutex = nullptr;

    bool begin() {
        mutex = xSemaphoreCreateMutex();
        if (mutex == nullptr) {
            return false;
        }
        return true;
    }

    GlobalState get() {
        GlobalState copy;
        xSemaphoreTake(mutex, portMAX_DELAY);
        copy = globalState;
        xSemaphoreGive(mutex);
        return copy;
    }

    bool setRadio(RemoteData& data) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.remoteData = data;
        globalState.radioConnected = true;
        globalState.lastFrameReceivedAt = millis();
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setAction(Action action) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.action = action;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setLowSpeedMode(bool lowSpeed) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.lowSpeedMode = lowSpeed;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setIsYellow(bool isYellow) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.isYellowTeam = isYellow;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setStocking(bool isStocking) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.isStocking = isStocking;
        xSemaphoreGive(mutex);
        
        return true;
    }
}