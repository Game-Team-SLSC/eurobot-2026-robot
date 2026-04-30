#pragma once

#include <commands.h>
#include <cstdint>
#include <battery.h>

namespace robot::screen {
    enum class Tab: uint8_t {
        DASHBOARD,
        LOGS,
        BATTERY
    };

    bool begin();
    bool focus(Tab tab);
    void updateStatus(const char* status);
    void updateControl(const char* control);
    void updateRemoteFreq(uint8_t freq);
    void updateBatteryPercentage(uint8_t percentage);
    void updateTeamColor(bool isYellow);

    void updateBatteryStatus(robot::battery::BatteryStatus &status);

    void updateLogs(const char logs[10][128]);
}