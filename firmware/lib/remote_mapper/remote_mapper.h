#pragma once

#include "types.h"

namespace robot::remote_mapper {

JoystickCommand ToJoystickCommand(const RemoteData& frame, uint32_t sequence);

}  // namespace robot::remote_mapper
