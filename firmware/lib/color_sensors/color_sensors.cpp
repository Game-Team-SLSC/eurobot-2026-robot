#include "i2cexpander.h"

#include <Arduino.h>
#include <DFRobot_TCS34725.h>
#include <FreeRTOS.h>
#include <freertos/queue.h>
#include <queues.h>
#include <Logger.h>
#include <buses.h>
#include <config.h>

namespace {
    DFRobot_TCS34725 ferSensor(robot::buses::get(robot::config::I2CBusId::SENSORS));
    DFRobot_TCS34725 firSensor(robot::buses::get(robot::config::I2CBusId::SENSORS));
    DFRobot_TCS34725 filSensor(robot::buses::get(robot::config::I2CBusId::SENSORS));
    DFRobot_TCS34725 felSensor(robot::buses::get(robot::config::I2CBusId::SENSORS));
}

struct HSV {
    float h;  // 0 à 360 degrés
    float s;  // 0 à 1
    float v;  // 0 à 1
};

HSV rgbToHsv(uint16_t R, uint16_t G, uint16_t B) {
    const float r = static_cast<float>(R) / 65535.0f;
    const float g = static_cast<float>(G) / 65535.0f;
    const float b = static_cast<float>(B) / 65535.0f;

    const float cmax = max(r, max(g, b));
    const float cmin = min(r, min(g, b));
    const float delta = cmax - cmin;

    HSV hsv{};
    hsv.v = cmax;
    hsv.s = (cmax <= 1e-6f) ? 0.0f : (delta / cmax);

    // Hue peu fiable quand saturation quasi nulle
    if (delta <= 1e-6f) {
        hsv.h = 0.0f;
        return hsv;
    }

    if (cmax == r) {
        hsv.h = 60.0f * fmod(((g - b) / delta), 6.0f);
    } else if (cmax == g) {
        hsv.h = 60.0f * (((b - r) / delta) + 2.0f);
    } else {
        hsv.h = 60.0f * (((r - g) / delta) + 4.0f);
    }

    if (hsv.h < 0.0f) {
        hsv.h += 360.0f;
    }

    return hsv;
}

bool beginTarget(DFRobot_TCS34725& target) {
    const bool beginOk = target.begin();
    target.setGain(TCS34725_GAIN_60X);

    vTaskDelay(pdMS_TO_TICKS(200));

    return beginOk;
}

bool readTarget(DFRobot_TCS34725& target) {
    uint16_t r, g, b, c;
    target.getRGBC(&r, &g, &b, &c);

    HSV hsv = rgbToHsv(r, g, b);
    ColorResponse cmd{.h = hsv.h, .s = hsv.s, .v = hsv.v};
    xQueueSend(robot::queues::color_response_queue, &cmd, 0);

    return true;
}

void focusI2COnTarget(const robot::config::SubI2CDeviceConfig& target) {
    I2CExpanderCommand cmd;
    cmd.controller = target.controller;
    cmd.channel = target.channel;
    robot::i2cexpander::apply(cmd);
}

namespace robot::color_sensors {
bool begin() {
    bool ok = true;

    focusI2COnTarget(robot::config::FER_CAPTOR);
    if (!beginTarget(ferSensor)) {
        ok = false;
    }

        focusI2COnTarget(robot::config::FIR_CAPTOR);
    if (!beginTarget(firSensor)) {
        ok = false;
    }

    focusI2COnTarget(robot::config::FIL_CAPTOR);
    if (!beginTarget(filSensor)) {
        ok = false;
    }

    focusI2COnTarget(robot::config::FEL_CAPTOR);
    if (!beginTarget(felSensor)) {
        ok = false;
    }

    if (!ok) {
        error("color_sensors", "Initialization failed");
        return false;
    }

    info("color_sensors", "Initialized");

    return ok;
}

bool apply(const ColorCommand& command) {
    DFRobot_TCS34725* target = nullptr;
    const robot::config::SubI2CDeviceConfig* cfg = nullptr;

    switch (command.sensor) {
        case robot::config::ColorSensor::FER: cfg = &robot::config::FER_CAPTOR; target = &ferSensor; break;
        case robot::config::ColorSensor::FIR: cfg = &robot::config::FIR_CAPTOR; target = &firSensor; break;
        case robot::config::ColorSensor::FIL: cfg = &robot::config::FIL_CAPTOR; target = &filSensor; break;
        case robot::config::ColorSensor::FEL: cfg = &robot::config::FEL_CAPTOR; target = &felSensor; break;
        default:
            error("color_sensors", "Invalid color sensor : %u", static_cast<unsigned int>(command.sensor));
            return false;
    }

    focusI2COnTarget(*cfg);

    // Optionnel: jeter une 1ère lecture
    uint16_t r, g, b, c;
    target->getRGBC(&r, &g, &b, &c);

    return readTarget(*target);
}

} // namespace robot::color_sensors