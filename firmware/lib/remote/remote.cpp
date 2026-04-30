#include "remote.h"

#include <secrets.h>
#include <config.h>
#include <SPI.h>
#include <cstring>
#include <Logger.h>
#include <spi_mutex.h>

namespace {
RF24 *radio = nullptr;
constexpr uint32_t NO_RX_LOG_PERIOD_MS = 2000;
constexpr uint32_t RF24_SPI_HZ = 2000000;

void setAllTmcChipSelectInactive() {
	pinMode(robot::config::tmc_fr_config.csPin, OUTPUT);
	digitalWrite(robot::config::tmc_fr_config.csPin, HIGH);
	pinMode(robot::config::tmc_fl_config.csPin, OUTPUT);
	digitalWrite(robot::config::tmc_fl_config.csPin, HIGH);
	pinMode(robot::config::tmc_br_config.csPin, OUTPUT);
	digitalWrite(robot::config::tmc_br_config.csPin, HIGH);
	pinMode(robot::config::tmc_bl_config.csPin, OUTPUT);
	digitalWrite(robot::config::tmc_bl_config.csPin, HIGH);
}

uint64_t rfAddressToUint64(const char* addr) {
	uint64_t packed = 0;
	for (uint8_t i = 0; (i < 5U) && (addr[i] != '\0'); ++i) {
		packed |= (static_cast<uint64_t>(static_cast<uint8_t>(addr[i])) << (8U * i));
	}
	return packed;
}

}

namespace robot::remote {
	bool connect() {
		robot::spi_mutex::Guard spiGuard;
		if (!spiGuard.isLocked()) {
			error("remote", "SPI mutex unavailable");
			return false;
		}

		// Keep RF24 deselected while configuring the shared SPI bus.
		pinMode(robot::config::rf_csn_pin, OUTPUT);
		digitalWrite(robot::config::rf_csn_pin, HIGH);
		pinMode(robot::config::rf_ce_pin, OUTPUT);
		digitalWrite(robot::config::rf_ce_pin, LOW);

		setAllTmcChipSelectInactive();
		
		if (radio == nullptr) {
			radio = new RF24(robot::config::rf_ce_pin, robot::config::rf_csn_pin, RF24_SPI_HZ);
		}

		radio->flush_rx();
		radio->flush_tx();
		radio->clearStatusFlags();

		// (SPI is initialized in buses::begin)
		const bool ready = radio->begin(&SPI);
		const bool chipConnected = radio->isChipConnected();
		if (!ready) {
			error("remote", "RF24 begin failed");
			return false;
		}

		radio->openReadingPipe(1, rfAddressToUint64(robot::secrets::rf_address));
		radio->setPALevel(RF24_PA_HIGH);
		radio->setDataRate(RF24_250KBPS);
		radio->setChannel(robot::secrets::rf_channel);
			
		radio->startListening();

		info("remote", "Initialized");
		return true;
	}

	bool fetch(RemoteData& data) {
		if (radio == nullptr) {
			warn("remote", "fetch called before radio initialized");
			return false;
		}

		robot::spi_mutex::Guard spiGuard;
		if (!spiGuard.isLocked()) {
			return false;
		}

		if (!radio->available()) {
			return false;
		}

		radio->read(&data, sizeof(data));

		return true;
	}
}
