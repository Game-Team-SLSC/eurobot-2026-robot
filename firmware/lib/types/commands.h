#pragma once

#include <cstdint>
#include <config.h>

constexpr uint8_t MAX_CMD = 8;

struct MotionCommand {
    float forward = 0;
    float strafe = 0;
    float rotate = 0;

    struct {
        int16_t forward = 0;
        int16_t strafe = 0;
        int16_t rotate = 0;
    } target;

    int16_t maxSpeed = 0;
    
    uint32_t timestampMs = 0;
};

enum class IOAction: uint8_t {
    READ,
    WRITE
};
struct IOExpanderCommand {
    robot::config::IOExpander expander;
    uint8_t pin = 0;
    IOAction action = IOAction::WRITE;
    bool level = false;
};

struct PWMCommand {
    robot::config::PWMController controller;
    uint8_t pin;
    uint16_t value = 0;
};

struct ColorCommand {
    robot::config::ColorSensor sensor;
};

struct I2CExpanderCommand {
    robot::config::I2CController controller;
    uint8_t channel;
};

struct ColorResponse {
    float h;
    float s;
    float v;
};