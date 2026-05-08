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
    
    bool setRadioFrequency(uint8_t frequency) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.radioFrequency = frequency;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setBatteryStatus(robot::battery::BatteryStatus status) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.batteryStatus = status;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setPumpsStatus(bool enabled) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.pumpsOn = enabled;
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

    bool setFrontStocking(StockingState stockingState) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.front_stocking_state = stockingState;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setBackStocking(StockingState stockingState) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.back_stocking_state = stockingState;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setSpeedGain(float gain) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.speedGain = gain;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setCriticalBattery(bool critical) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.criticalBattery = critical;
        xSemaphoreGive(mutex);
        
        return true;
    }

    bool setEncoder(int16_t value) {
        if (!xSemaphoreTake(mutex, portMAX_DELAY)) {
            return false;
        }

        globalState.encoderPosition = value;
        xSemaphoreGive(mutex);
        
        return true;
    }
}