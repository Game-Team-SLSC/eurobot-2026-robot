#pragma once

#include <cstdint>
#include <FreeRTOS.h>
#include <freertos/semphr.h>
#include <RemoteData.h>
#include <config.h>

struct GlobalState {
    RemoteData remoteData{};
    bool radioConnected = false;
    robot::config::Action action = robot::config::Action::IDLE;
    uint32_t lastFrameReceivedAt = 0;
    bool lowSpeedMode = false;
    
};

namespace robot::state {

    extern SemaphoreHandle_t mutex;

    bool begin();

    GlobalState get();

    // setters

    bool setRadio(RemoteData& data);
    bool setAction(robot::config::Action action);
    bool setLowSpeedMode(bool lowSpeed);
}
