#pragma once

#include <cstdint>

#include <commands.h>

namespace robot::actions::detail {
void togglePumps(uint8_t state);
void angleTurn(CommandBatch<PWMCommand>& batch, uint8_t angle);
bool isOurTeam(const ColorResponse& color);
uint16_t angleToPWMValue(uint8_t angle);
}