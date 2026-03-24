#pragma once

#include <cstdint>

namespace robot::config {

constexpr uint32_t SERIAL_BAUDRATE = 115200;

// Task periods in milliseconds.
constexpr uint32_t COMM_PERIOD_MS = 10;      // 100 Hz base loop (supports up to 200 Hz RF updates)
constexpr uint32_t SENSOR_PERIOD_MS = 20;    // 50 Hz sensor aggregation
constexpr uint32_t CONTROL_PERIOD_MS = 10;   // 100 Hz control loop
constexpr uint32_t UI_PERIOD_MS = 100;       // 10 Hz UI refresh
constexpr uint32_t SAFETY_PERIOD_MS = 50;    // 20 Hz safety checks

// Event queue sizing.
constexpr uint32_t EVENT_QUEUE_LENGTH = 32;

// RF24 link supervision.
constexpr uint32_t RF_LINK_TIMEOUT_MS = 250;

// RF24 radio setup.
constexpr uint8_t RF24_CHANNEL = 90;
constexpr uint8_t RF24_DATA_RATE = 1;  // 0: 250kbps, 1: 1Mbps, 2: 2Mbps
constexpr uint8_t RF24_PA_LEVEL = 1;   // 0: MIN, 1: LOW, 2: HIGH, 3: MAX
constexpr uint64_t RF24_RX_PIPE = 0xE8E8F0F0E1ULL;

// Battery display thresholds.
constexpr float BATTERY_WARN_VOLTAGE = 13.2F;

// Drive tuning.
constexpr int16_t DRIVE_MAX_LINEAR = 1000;
constexpr int16_t DRIVE_MAX_ANGULAR = 600;
constexpr uint16_t STEPPER_MAX_STEP_HZ = 4000;
constexpr uint16_t STEPPER_MIN_STEP_HZ = 40;

// I2C addresses.
constexpr uint8_t PCA9685_ADDR_0 = 0x40;
constexpr uint8_t PCA9685_ADDR_1 = 0x41;
constexpr uint8_t PCA9685_ADDR_2 = 0x42;
constexpr uint8_t TCA9555_PUMP_ADDR = 0x20;
constexpr uint8_t TCA9548_COLOR_ADDR = 0x70;
constexpr uint8_t COLOR_SENSOR_ADDR = 0x29;

// Servo pulse defaults (PCA9685 ticks at 50Hz cycle 0..4095).
constexpr uint16_t SERVO_PULSE_MIN = 120;
constexpr uint16_t SERVO_PULSE_MID = 307;
constexpr uint16_t SERVO_PULSE_MAX = 500;
constexpr uint16_t STOCK_ISLAND_LOCKED_PULSE = 220;
constexpr uint16_t STOCK_ISLAND_OPEN_PULSE = 420;
constexpr uint16_t FLIP_STRAIGHT_PULSE = 260;
constexpr uint16_t FLIP_FOLD_PULSE = 420;
constexpr uint16_t FLIP_LOCKED_PULSE = 220;
constexpr uint16_t FLIP_UNLOCKED_PULSE = 430;

// Shared PCA channels for auxiliary outputs.
constexpr uint8_t LED_PCA_INDEX = 2;
constexpr uint8_t LED_PCA_CHANNEL = 0;
constexpr uint8_t FAN_PCA_INDEX = 2;
constexpr uint8_t FAN_PCA_CHANNEL = 1;

// Color detection policy for TCS34725/SEN0201.
constexpr float COLOR_BLUE_RATIO_MIN = 1.15F;
constexpr float COLOR_YELLOW_RATIO_MIN = 1.10F;

}  // namespace robot::config
