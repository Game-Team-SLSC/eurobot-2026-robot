#pragma once

#include <cstdint>
#include <FreeRTOS.h>
#include <freertos/semphr.h>
#include <RemoteData.h>
#include <actions.h>

enum class StockingState: uint8_t {
    EMPTY,
    HALF,
    FULL
};

struct GlobalState {
    RemoteData remoteData{};
    bool radioConnected = false;
    uint32_t lastFrameReceivedAt = 0;

    float speedGain = 1.0f;

    Action action = Action::IDLE;

    bool criticalBattery = false;

    bool lowSpeedMode = false;
    bool isYellowTeam = false;
    StockingState stockingState = StockingState::EMPTY;
};

namespace robot::state {

    extern SemaphoreHandle_t mutex;

    bool begin();

    GlobalState get();

    // setters

    bool setRadio(RemoteData& data);
    bool setAction(Action action);
    bool setLowSpeedMode(bool lowSpeed);
    bool setIsYellow(bool isYellow);
    bool setStocking(StockingState stockingState);
    bool setSpeedGain(float gain);
    bool setCriticalBattery(bool critical);
}
