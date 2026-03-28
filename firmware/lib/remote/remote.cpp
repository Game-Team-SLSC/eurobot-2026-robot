#include "remote.h"

#include <secrets.h>
#include <config.h>

namespace {
RF24 radio(robot::config::rf_ce_pin, robot::config::rf_csn_pin);
}

namespace robot::remote {
	bool connect() {
		if (!radio.begin()) {
			return false;
		}

		radio.openReadingPipe(1, robot::secrets::rf_address);
		radio.setChannel(robot::secrets::rf_channel);
		radio.setDataRate(RF24_250KBPS);
		radio.setPALevel(RF24_PA_LOW);

		radio.startListening();

		return true;
	}

	bool fetch(robot::types::RemoteData& data) {
		if (!radio.available()) {
			return false;
		}
		
		
		radio.read(&data, sizeof(robot::types::RemoteData));
		return true;
	}
}
