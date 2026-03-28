#pragma once

#include <cstdint>

namespace robot::state {
    struct MotionCommand {
        int8_t forward = 0;
        int8_t strafe = 0;
        int8_t rotate = 0;
        uint32_t timestampMs = 0;
    };

    struct GlobalState {
        bool radioConnected = false;
        bool radioTimedOut = true;
        uint32_t lastFrameTimestampMs = 0;
        MotionCommand activeCommand{};
    };

    bool begin();
    bool get(GlobalState& out);

    bool setRadioConnected(bool connected);
    bool setFrameReceivedAt(uint32_t timestampMs);
    bool setControlSnapshot(bool timedOut, const MotionCommand& command);
}
