#pragma once

#include <cstdint>

#include <ioexpander.h>
#include <state.h>

namespace robot::tasks {
    using MotionCommand = robot::state::MotionCommand;
    using GlobalState = robot::state::GlobalState;

    bool begin();
    bool getGlobalState(GlobalState& out);
    bool submitIoCommand(const robot::ioexpander::Command& command);

    void commTask(void* parameter);
    void controlTask(void* parameter);
    void ioTask(void* parameter);
}