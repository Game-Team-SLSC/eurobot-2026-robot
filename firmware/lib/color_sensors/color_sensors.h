#pragma once

#include <config.h>
#include <cstdint>
#include <commands.h>

namespace robot::color_sensors {

	bool begin();
	bool apply(const ColorCommand& command);
}
