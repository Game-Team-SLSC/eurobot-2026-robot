#include "remote.h"

#include <secrets.h>
#include <config.h>
#include <logging.h>
#include <SPI.h>
#include <cstring>

namespace {
RF24 *radio = nullptr;
SPIClass radioSpi(HSPI);
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

		radioSpi.begin(robot::config::spi_sck_pin,
		              robot::config::spi_miso_pin,
		              robot::config::spi_mosi_pin);
					  
		radioSpi.setFrequency(RF24_SPI_HZ);
		
		const bool ready = radio->begin(&radioSpi);
		const bool chipConnected = radio->isChipConnected();
		if (!ready) {
			robot::logging::warn("remote", "radio.begin failed");
			robot::logging::warnf("remote", "isChipConnected=%u", static_cast<unsigned int>(chipConnected));
			return false;
		}

		robot::logging::infof("remote", "isChipConnected=%u", static_cast<unsigned int>(chipConnected));

		radio->openReadingPipe(1, rfAddressToUint64(robot::secrets::rf_address));
		radio->setDataRate(RF24_250KBPS);
		radio->setChannel(robot::secrets::rf_channel);
			
		radio->startListening();

		radio->printDetails();

		return true;
	}

	bool fetch(robot::types::RemoteData& data) {
		if (radio == nullptr) {
			Serial.println("[remote] fetch called before radio initialized");
			return false;
		}

		if (!radio->available()) {
			static uint32_t lastNoRxLogMs = 0;
			const uint32_t nowMs = millis();
			if ((nowMs - lastNoRxLogMs) >= NO_RX_LOG_PERIOD_MS) {
				lastNoRxLogMs = nowMs;
				robot::logging::infof("remote", "no packet yet carrier");
			}
			return false;
		}

		radio->read(&data, sizeof(data));

		Serial.println("[remote] frame received");
		return true;
	}
}
