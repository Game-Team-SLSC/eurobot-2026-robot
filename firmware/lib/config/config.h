#pragma once

#include <cstdint>

namespace robot::config {
    enum class I2CBusId: uint8_t {
        ACTUATION,
        SENSORS
    };

    enum class PWMController: uint8_t {
        MISC,
        LEFT,
        RIGHT
    };

    enum class Button: uint8_t {
        LSIDE_L_BTN,
        LSIDE_U_BTN,
        LSIDE_D_BTN,
        LSIDE_R_BTN,

        RSIDE_L_BTN,
        RSIDE_U_BTN,
        RSIDE_D_BTN,
        RSIDE_R_BTN,

        DOUBLE_U_BTN,
        DOUBLE_D_BTN,

        _BUTTON_COUNT
    };

    enum class Actions: uint8_t {
        TURN,

        _ACTION_COUNT
    };

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

struct PWMControl {
    robot::config::PWMController controller;
    uint8_t pin;
};
}


namespace robot::config {
    constexpr uint8_t critical_batt_th = 15; // %
    constexpr uint8_t warning_batt_th = 25; // %

    constexpr float cell_1_voltage_ratio = 1.67;
    constexpr float cell_2_voltage_ratio = 1.67;
    constexpr float cell_3_voltage_ratio = 1.67;
    constexpr float full_bat_voltage_ratio = 1.67;

    constexpr I2CBusConfig i2c_actuation_config = {13, 14};
    constexpr I2CBusConfig i2c_sensors_config = {4, 5};

    constexpr I2CDeviceConfig ads1015_i2c_config = {0x48, I2CBusId::SENSORS};
    constexpr I2CDeviceConfig logic_tca9548_i2c_config = {0x70, I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig tca9555_i2c_config = {0x27, I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_left_i2c_config = {-1,  I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_right_i2c_config = {-1,  I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_misc_i2c_config = {-1,  I2CBusId::ACTUATION};

    constexpr uint8_t tca_pin_motors_enable = 2;

    constexpr TMCConfig tmc_fr_config = {39, 41, 1, true};
    constexpr TMCConfig tmc_fl_config = {40, 42, 2};
    constexpr TMCConfig tmc_br_config = {9, 15, 18};
    constexpr TMCConfig tmc_bl_config = {8, 16, 17};
    
    constexpr float tmc_rsense = 0.075f;
    constexpr uint16_t motor_microsteps = 8;
    constexpr uint32_t motion_speed_hz = 15000;
    constexpr uint32_t motion_accel = 25000;
    constexpr uint16_t motor_rms_current_ma = 1200; 

    constexpr float movers_steps_per_meter = 200.0f / 0.1885f;
    constexpr float movers_velocity = 1.0f;
    constexpr uint16_t movers_control_hz = 100;

    constexpr uint8_t spi_sck_pin = 12;
    constexpr uint8_t spi_miso_pin = 11;
    constexpr uint8_t spi_mosi_pin = 21;

    constexpr uint8_t rf_ce_pin = 48;
    constexpr uint8_t rf_csn_pin = 38;
    constexpr uint8_t rf_frequency = 50; // Hz
    constexpr uint16_t rf_timeout_ms = 400;

    constexpr PWMControl back_left_turner = {robot::config::PWMController::LEFT, 8};
    constexpr PWMControl front_left_turner = {robot::config::PWMController::LEFT, 9};
    constexpr PWMControl back_right_turner = {robot::config::PWMController::RIGHT, 8};
    constexpr PWMControl front_right_turner = {robot::config::PWMController::RIGHT, 9};

    constexpr PWMControl back_left_grabber = {robot::config::PWMController::LEFT, 6};
    constexpr PWMControl front_left_grabber = {robot::config::PWMController::LEFT, 7};
    constexpr PWMControl back_right_grabber = {robot::config::PWMController::RIGHT, 6};
    constexpr PWMControl front_right_grabber = {robot::config::PWMController::RIGHT, 7};

    constexpr robot::config::Button turn_action_btn = robot::config::Button::LSIDE_U_BTN;
}