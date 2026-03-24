#include "radio.h"

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

#include "config.h"
#include "hardware_pins.h"

namespace robot::radio {

class RadioReceiver::Impl {
 public:
  Impl()
      : rf24(pins::spi::RF24_CE, pins::spi::RF24_CSN),
        ready(false) {}

  RF24 rf24;
  bool ready;
  CommandAck pendingAck;
  bool hasPendingAck = false;
};

namespace {

rf24_datarate_e ToDataRate(uint8_t mode) {
  switch (mode) {
    case 0:
      return RF24_250KBPS;
    case 2:
      return RF24_2MBPS;
    default:
      return RF24_1MBPS;
  }
}

rf24_pa_dbm_e ToPaLevel(uint8_t level) {
  switch (level) {
    case 0:
      return RF24_PA_MIN;
    case 2:
      return RF24_PA_HIGH;
    case 3:
      return RF24_PA_MAX;
    default:
      return RF24_PA_LOW;
  }
}

}  // namespace

RadioReceiver::RadioReceiver()
    : impl_(new Impl()) {}

bool RadioReceiver::begin() {
  if (pins::spi::RF24_CE < 0 || pins::spi::RF24_CSN < 0) {
    Serial.println("[radio] invalid RF24 pins, receiver disabled");
    impl_->ready = false;
    return false;
  }

  if (pins::spi::SCK >= 0 && pins::spi::MISO >= 0 && pins::spi::MOSI >= 0) {
    SPI.begin(pins::spi::SCK, pins::spi::MISO, pins::spi::MOSI, pins::spi::RF24_CSN);
  }

  if (!impl_->rf24.begin()) {
    Serial.println("[radio] RF24 begin failed");
    impl_->ready = false;
    return false;
  }

  impl_->rf24.setAutoAck(true);
  impl_->rf24.enableAckPayload();
  impl_->rf24.setRetries(3, 5);
  impl_->rf24.setChannel(config::RF24_CHANNEL);
  impl_->rf24.setDataRate(ToDataRate(config::RF24_DATA_RATE));
  impl_->rf24.setPALevel(ToPaLevel(config::RF24_PA_LEVEL));
  impl_->rf24.setPayloadSize(sizeof(RemoteData));
  impl_->rf24.openReadingPipe(1, config::RF24_RX_PIPE);
  impl_->rf24.startListening();

  impl_->ready = true;
  Serial.println("[radio] RF24 receiver ready (polling mode)");
  return true;
}

bool RadioReceiver::read(RemoteData* outFrame) {
  if (!impl_->ready || outFrame == nullptr) {
    return false;
  }

  bool gotFrame = false;
  uint8_t pipe = 0;
  while (impl_->rf24.available(&pipe)) {
    if (impl_->hasPendingAck) {
      impl_->rf24.writeAckPayload(pipe, &impl_->pendingAck, sizeof(CommandAck));
      impl_->hasPendingAck = false;
    }
    impl_->rf24.read(outFrame, sizeof(RemoteData));
    gotFrame = true;
  }
  return gotFrame;
}

void RadioReceiver::setAck(const CommandAck& ack) {
  impl_->pendingAck = ack;
  impl_->hasPendingAck = true;
}

bool RadioReceiver::isReady() const {
  return impl_->ready;
}

}  // namespace robot::radio
