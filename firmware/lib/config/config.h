#pragma once

#include <cstdint>

#define LOG_STATE
#define INFO_STATE
#define WARN_STATE
#define ERROR_STATE

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

    enum class IOExpander: uint8_t {
        LOGIC,
        KINETIC
    };

    enum class I2CController: uint8_t {
        TCA9548_SENSORS,
        TCA9548_LOGIC
    };

    enum class ColorSensor: uint8_t {
        FEL,
        FIL,
        FIR,
        FER
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

struct I2CBusConfig {
    uint8_t sdaPin;
    uint8_t sclPin;
};

struct I2CDeviceConfig {
    uint8_t address;
    robot::config::I2CBusId busId;
};

struct SubI2CDeviceConfig {
    uint8_t channel;
    robot::config::I2CController controller;
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
    // Battery
    constexpr uint8_t critical_batt_th = 1; // %
    constexpr uint8_t warning_batt_th = 25; // %

    constexpr float cell_1_voltage_ratio = 1.679;
    constexpr float cell_2_voltage_ratio = 3.371;
    constexpr float cell_3_voltage_ratio = 5.048;
    constexpr float full_bat_voltage_ratio = 6.687;

    // I2C config


    constexpr I2CBusConfig i2c_actuation_config = {13, 14};
    constexpr I2CBusConfig i2c_sensors_config = {4, 5};

    constexpr I2CDeviceConfig ads1015_i2c_config = {0x48, I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig tca9555_logic_i2c_config = {0x27, I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig tca9555_kinetic_i2c_config = {0x26, I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_left_i2c_config = {0x5f,  I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_right_i2c_config = {0x4f,  I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig pca9685_misc_i2c_config = {0x57,  I2CBusId::ACTUATION};
    constexpr I2CDeviceConfig tca9548_logic_i2c_config = {0x77, I2CBusId::SENSORS};

    // Movers config

    constexpr uint8_t tca_pin_motors_enable = 2;

    constexpr TMCConfig tmc_fr_config = {39, 41, 1, true};
    constexpr TMCConfig tmc_fl_config = {40, 42, 2};
    constexpr TMCConfig tmc_br_config = {9, 15, 18};
    constexpr TMCConfig tmc_bl_config = {8, 16, 17};
    
    constexpr float tmc_rsense = 0.075f;
    constexpr uint16_t motor_microsteps = 8;
    constexpr uint32_t motion_speed_hz = 9000;
    constexpr uint32_t motion_accel = 22500;
    constexpr uint16_t motor_rms_current_ma = 2000; 

    constexpr float wheel_radius_mm = 30.0f;
    constexpr float mm_to_m = 0.001f;
    constexpr float pi_f = 3.14159265f;
    constexpr float wheel_circumference_m = 2.0f * pi_f * wheel_radius_mm * mm_to_m;
    constexpr float motor_steps_per_revolution = 200.0f * static_cast<float>(motor_microsteps);
    constexpr float movers_steps_per_meter = motor_steps_per_revolution / wheel_circumference_m;
    constexpr float movers_velocity = 1.0f;
    constexpr uint16_t movers_control_hz = 100;

    constexpr float min_speed_gain = 0.1f;
    constexpr float max_speed_gain = 1.0f;

    // SPI config

    constexpr uint8_t spi_sck_pin = 12;
    constexpr uint8_t spi_miso_pin = 11;
    constexpr uint8_t spi_mosi_pin = 21;

    // Radio config

    constexpr uint8_t rf_ce_pin = 48;
    constexpr uint8_t rf_csn_pin = 38;
    constexpr uint8_t rf_frequency = 50; // Hz
    constexpr uint16_t rf_timeout_ms = 400;

    // Servo config

    constexpr PWMControl back_left_turner = {robot::config::PWMController::LEFT, 8};
    constexpr PWMControl front_left_turner = {robot::config::PWMController::LEFT, 8};
    constexpr PWMControl back_right_turner = {robot::config::PWMController::RIGHT, 8};
    constexpr PWMControl front_right_turner = {robot::config::PWMController::RIGHT, 8};

    constexpr PWMControl back_left_grabber = {robot::config::PWMController::LEFT, 6};
    constexpr PWMControl front_left_grabber = {robot::config::PWMController::LEFT, 7};
    constexpr PWMControl back_right_grabber = {robot::config::PWMController::RIGHT, 6};
    constexpr PWMControl front_right_grabber = {robot::config::PWMController::RIGHT, 7};

    constexpr PWMControl led_left_b = {robot::config::PWMController::LEFT, 0};
    constexpr PWMControl led_left_g = {robot::config::PWMController::LEFT, 1};
    constexpr PWMControl led_left_r = {robot::config::PWMController::LEFT, 2};

    // Color config

    constexpr SubI2CDeviceConfig FEL_CAPTOR = {3, I2CController::TCA9548_LOGIC};
    constexpr SubI2CDeviceConfig FIL_CAPTOR = {2, I2CController::TCA9548_LOGIC};
    constexpr SubI2CDeviceConfig FIR_CAPTOR = {1, I2CController::TCA9548_LOGIC};
    constexpr SubI2CDeviceConfig FER_CAPTOR = {0, I2CController::TCA9548_LOGIC};

    // Diag LEDs

    // TCA9555 pin indices are 0..15:
    // P14=12, P15=13, P16=14, P17=15.
    constexpr uint8_t led_1_tca_pin = 15; // P17
    constexpr uint8_t led_2_tca_pin = 14; // P16
    constexpr uint8_t led_3_tca_pin = 13; // P15
    constexpr uint8_t led_4_tca_pin = 12; // P14

    // Actions config

    constexpr robot::config::Button turn_action_btn = robot::config::Button::LSIDE_U_BTN;
    constexpr robot::config::Button stock_action_btn = robot::config::Button::LSIDE_R_BTN;
    constexpr robot::config::Button release_action_btn = robot::config::Button::LSIDE_D_BTN;
    constexpr robot::config::Button turn_two_action_btn = robot::config::Button::LSIDE_L_BTN;
    
    constexpr robot::config::Button toggle_front_grabber_btn = robot::config::Button::RSIDE_R_BTN;
    constexpr robot::config::Button toggle_back_grabber_btn = robot::config::Button::DOUBLE_D_BTN;

    constexpr robot::config::Button yellow_mode_btn = robot::config::Button::RSIDE_U_BTN;
    constexpr robot::config::Button blue_mode_btn = robot::config::Button::RSIDE_L_BTN;
}