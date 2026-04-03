#pragma once

#include <config.h>
#include <cstdint>
#include <commands.h>

namespace robot::i2cexpander {

	bool begin();
	bool apply(const I2CExpanderCommand& command);
	bool apply(const CommandBatch<I2CExpanderCommand>& batch);
}
