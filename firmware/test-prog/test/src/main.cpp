#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <types.h>
#include <config.h>
constexpr uint8_t RADIO_CE_PIN = 48;
constexpr uint8_t RADIO_CSN_PIN = 38;
namespace {
SPIClass g_hspi(HSPI);
RF24* g_radio = nullptr;

constexpr uint8_t TMC_CS_FR_PIN = 39;
constexpr uint8_t TMC_CS_FL_PIN = 40;
constexpr uint8_t TMC_CS_BR_PIN = 9;
constexpr uint8_t TMC_CS_BL_PIN = 8;

uint64_t rfAddressToUint64(const char* addr) {
  uint64_t packed = 0;
  for (uint8_t i = 0; (i < 5U) && (addr[i] != '\0'); ++i) {
    packed |= (static_cast<uint64_t>(static_cast<uint8_t>(addr[i])) << (8U * i));
  }
  return packed;
}
} // namespace

namespace robot::radio_link {
void prepare(uint8_t cePin, uint8_t csnPin) {
  pinMode(csnPin, OUTPUT);
  digitalWrite(csnPin, HIGH);
  pinMode(cePin, OUTPUT);
  digitalWrite(cePin, LOW);
}

bool begin(uint8_t sckPin,
           uint8_t misoPin,
           uint8_t mosiPin,
           uint8_t cePin,
           uint8_t csnPin,
           const char* address,
           uint8_t channel,
           uint32_t spiHz) {
  if (g_radio == nullptr) {
    g_radio = new RF24(cePin, csnPin, spiHz);
  }

  g_hspi.begin(sckPin, misoPin, mosiPin);
  g_hspi.setFrequency(spiHz);

  const bool ready = g_radio->begin(&g_hspi);
  if (!ready) {
    Serial.println("[RADIO] begin failed, continuing with forced config");
    Serial.print("[RADIO] isChipConnected: ");
    Serial.println(g_radio->isChipConnected());
  }

  g_radio->openReadingPipe(1, rfAddressToUint64(address));
  g_radio->setDataRate(RF24_250KBPS);
  g_radio->setChannel(channel);
  g_radio->startListening();

  Serial.printf("[RADIO] ready=%u channel=%u spi=%luHz\n",
                static_cast<unsigned int>(ready),
                static_cast<unsigned int>(channel),
                static_cast<unsigned long>(spiHz));
  g_radio->printDetails();

  return ready;
}

bool receive(robot::types::RemoteData& data) {
  if ((g_radio == nullptr) || !g_radio->available()) {
    return false;
  }

  g_radio->read(&data, sizeof(data));
  while (g_radio->available()) {
    // Keep the latest frame if multiple payloads queued.
    g_radio->read(&data, sizeof(data));
  }

  return true;
}
} // namespace robot::radio_link

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  robot::radio_link::begin(robot::config::spi_sck_pin,
                            robot::config::spi_miso_pin,
                            robot::config::spi_mosi_pin,
                            RADIO_CE_PIN,
                            RADIO_CSN_PIN,
                            "GT912",
                            100,
                            2000000);
}

void loop() {
  robot::types::RemoteData data;
  if (robot::radio_link::receive(data)) {
    Serial.print("Received data: ");
    Serial.print("Left joystick: (");
    Serial.print(data.joystickLeft.x);
    Serial.print(", ");
    Serial.print(data.joystickLeft.y);
    Serial.print(") | Right joystick: (");
    Serial.print(data.joystickRight.x);
    Serial.print(", ");
    Serial.print(data.joystickRight.y);
    Serial.print(") | Buttons: [");
    for (size_t i = 0; i < sizeof(data.buttons) / sizeof(data.buttons[0]); ++i) {
      Serial.print(data.buttons[i] ? "1" : "0");
      if (i < sizeof(data.buttons) / sizeof(data.buttons[0]) - 1) {
        Serial.print(", ");
      }
    }
    Serial.print("] | Slider: ");
    Serial.print(data.slider);
    Serial.print(" | Score: ");
    Serial.println(data.score);
  }

  delay(100);
}