#pragma once

#include <cstdint>

namespace robot::config {
    enum class I2CBusId: uint8_t {
        ACTUATION = 0,
        SENSORS = 1
    };
}

struct I2CBusConfig {
    uint8_t sdaPin;
    uint8_t sclPin;
};

struct I2CDeviceConfig {
    uint8_t address;
    robot::config::I2CBusId busId;
};

struct SPIBusConfig {
    uint8_t sckPin;
    uint8_t misoPin;
    uint8_t mosiPin;
};

struct TMCConfig {
    uint8_t csPin;
    uint8_t stepPin;
    uint8_t dirPin;
    bool dirHighCountsUp = false;
};

namespace robot::config {
    constexpr uint8_t critical_batt_th = 15; // %
    constexpr uint8_t warning_batt_th = 25; // %

    constexpr float cell_1_voltage_ratio = 1.67;
    constexpr float cell_2_voltage_ratio = 1.67;
    constexpr float cell_3_voltage_ratio = 1.67;
    constexpr float full_bat_voltage_ratio = 1.67;
    
    constexpr float tmc_rsense = 0.075f;
    constexpr uint16_t motor_microsteps = 8;
    constexpr uint32_t motion_speed_hz = 15000;
    constexpr uint32_t motion_accel = 25000;
    constexpr uint16_t motor_rms_current_ma = 1200; 

    // Movement calibration: 200 steps = 18.85 cm.
    constexpr float movers_steps_per_meter = 200.0f / 0.1885f;
    // Target robot linear speed in m/s at full joystick command.
    constexpr float movers_velocity = 1.0f;
    // drive() update frequency (Hz). Keep in sync with the control loop period.
    constexpr uint16_t movers_control_hz = 100;

    constexpr uint8_t spi_sck_pin = 12;
    constexpr uint8_t spi_miso_pin = 11;
    constexpr uint8_t spi_mosi_pin = 21;

    constexpr uint8_t rf_ce_pin = 48;
    constexpr uint8_t rf_csn_pin = 38;
    constexpr uint8_t rf_frequency = 50; // Hz
    constexpr uint16_t rf_timeout_ms = 400;
}