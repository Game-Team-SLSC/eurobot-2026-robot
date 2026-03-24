#pragma once

#include <cstdint>

namespace robot::pins {

// Set an entry to -1 when the hardware is not yet assigned.

namespace spi {
constexpr int8_t SCK = -1;
constexpr int8_t MISO = -1;
constexpr int8_t MOSI = -1;

constexpr int8_t RF24_CE = -1;
constexpr int8_t RF24_CSN = -1;
constexpr int8_t RF24_IRQ = -1;

constexpr int8_t DISPLAY_CS = -1;
constexpr int8_t DISPLAY_DC = -1;
constexpr int8_t DISPLAY_RST = -1;

constexpr int8_t TMC_CSN = -1;  // Shared by chain or replaced by per-driver CS pins.
}  // namespace spi

namespace i2c {
// I2C bus dedicated to sensors (TOF mux + color mux + ADS1015).
constexpr int8_t SENSORS_SDA = -1;
constexpr int8_t SENSORS_SCL = -1;

// I2C bus dedicated to actuators/expanders (PCA9685 + TCA9555).
constexpr int8_t ACTUATORS_SDA = -1;
constexpr int8_t ACTUATORS_SCL = -1;
}  // namespace i2c

namespace encoder {
constexpr int8_t A = -1;
constexpr int8_t B = -1;
constexpr int8_t SWITCH = -1;
}  // namespace encoder

namespace stepper {
constexpr int8_t FL_STEP = -1;
constexpr int8_t FL_DIR = -1;
constexpr int8_t FR_STEP = -1;
constexpr int8_t FR_DIR = -1;
constexpr int8_t RL_STEP = -1;
constexpr int8_t RL_DIR = -1;
constexpr int8_t RR_STEP = -1;
constexpr int8_t RR_DIR = -1;
}  // namespace stepper

}  // namespace robot::pins
