#pragma once

#include <cstdint>
#include <FreeRTOS.h>
#include <freertos/semphr.h>
#include <RemoteData.h>
#include <actions.h>

struct GlobalState {
    RemoteData remoteData{};
    bool radioConnected = false;
    uint32_t lastFrameReceivedAt = 0;

    Action action = Action::IDLE;

    bool lowSpeedMode = false;
    bool isYellowTeam = false;
    bool isStocking = false;
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
    bool setStocking(bool isStocking);
}
