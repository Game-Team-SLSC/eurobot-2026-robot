#pragma once

#include <cstdint>
#include <commands.h>

namespace robot::ioexpander {
    bool begin();
    bool apply(const IOExpanderCommand& command);
}