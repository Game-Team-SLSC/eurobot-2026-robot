#include "movers.h"

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#include <config.h>

namespace {

FastAccelStepperEngine engine = FastAccelStepperEngine();

FastAccelStepper* stepper_br = nullptr;
FastAccelStepper* stepper_bl = nullptr;
FastAccelStepper* stepper_fr = nullptr;
FastAccelStepper* stepper_fl = nullptr;

TMC5160Stepper driver_fr(robot::config::tmc_fr_config.csPin,
						 robot::config::tmc_rsense,
						 robot::config::spi_mosi_pin,
						 robot::config::spi_miso_pin,
						 robot::config::spi_sck_pin,
						 -1);
TMC5160Stepper driver_fl(robot::config::tmc_fl_config.csPin,
						 robot::config::tmc_rsense,
						 robot::config::spi_mosi_pin,
						 robot::config::spi_miso_pin,
						 robot::config::spi_sck_pin,
						 -1);
TMC5160Stepper driver_br(robot::config::tmc_br_config.csPin,
						 robot::config::tmc_rsense,
						 robot::config::spi_mosi_pin,
						 robot::config::spi_miso_pin,
						 robot::config::spi_sck_pin,
						 -1);
TMC5160Stepper driver_bl(robot::config::tmc_bl_config.csPin,
						 robot::config::tmc_rsense,
						 robot::config::spi_mosi_pin,
						 robot::config::spi_miso_pin,
						 robot::config::spi_sck_pin,
						 -1);

int32_t fr_target = 0;
int32_t fl_target = 0;
int32_t br_target = 0;
int32_t bl_target = 0;

constexpr int32_t JOYSTICK_MAX_ABS = 127;

static int32_t computeDriveStepDeltaMax() {
	const float stepsPerSecond = robot::config::movers_velocity * robot::config::movers_steps_per_meter;
	const float stepsPerTick = stepsPerSecond / static_cast<float>(robot::config::movers_control_hz);
	const int32_t rounded = static_cast<int32_t>(stepsPerTick + 0.5f);
	return max<int32_t>(1, rounded);
}

void applyDriverSettings(TMC5160Stepper& drv) {
	drv.begin();
	// drv.toff(4);
	// drv.blank_time(24);
	drv.rms_current(robot::config::motor_rms_current_ma);
	drv.microsteps(robot::config::motor_microsteps);
}

void applyStepperSettings(FastAccelStepper* stepper, const TMCConfig& axisConfig) {
	if (stepper == nullptr) {
		return;
	}

	stepper->setDirectionPin(axisConfig.dirPin, axisConfig.dirHighCountsUp);
    stepper->setSpeedInHz(robot::config::motion_speed_hz);
    stepper->setAcceleration(robot::config::motion_accel);
    stepper->forceStopAndNewPosition(0);
}
} // namespace

namespace robot::movers {

bool begin() {
	pinMode(robot::config::tmc_bl_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_br_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_fl_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_fr_config.csPin, OUTPUT);
    
    digitalWrite(robot::config::tmc_bl_config.csPin, HIGH);
    digitalWrite(robot::config::tmc_fl_config.csPin, HIGH);
    digitalWrite(robot::config::tmc_br_config.csPin, HIGH);
    digitalWrite(robot::config::tmc_fr_config.csPin, HIGH);

    applyDriverSettings(driver_fr);
    applyDriverSettings(driver_fl);
    applyDriverSettings(driver_br);
    applyDriverSettings(driver_bl);

	engine.init();

	stepper_fr = engine.stepperConnectToPin(robot::config::tmc_fr_config.stepPin);
	stepper_fl = engine.stepperConnectToPin(robot::config::tmc_fl_config.stepPin);
	stepper_br = engine.stepperConnectToPin(robot::config::tmc_br_config.stepPin);
	stepper_bl = engine.stepperConnectToPin(robot::config::tmc_bl_config.stepPin);

	if ((stepper_fr == nullptr) || (stepper_fl == nullptr) ||
		(stepper_br == nullptr) || (stepper_bl == nullptr)) {
		return false;
	}

	applyStepperSettings(stepper_fr, robot::config::tmc_fr_config);
    applyStepperSettings(stepper_fl, robot::config::tmc_fl_config);
    applyStepperSettings(stepper_br, robot::config::tmc_br_config);
    applyStepperSettings(stepper_bl, robot::config::tmc_bl_config);

	return true;
}

void drive(int8_t forward, int8_t strafe, int8_t rotate) {
	int32_t frCmd = static_cast<int32_t>(forward) - static_cast<int32_t>(strafe) - static_cast<int32_t>(rotate);
	int32_t flCmd = static_cast<int32_t>(forward) + static_cast<int32_t>(strafe) + static_cast<int32_t>(rotate);
	int32_t brCmd = static_cast<int32_t>(forward) + static_cast<int32_t>(strafe) - static_cast<int32_t>(rotate);
	int32_t blCmd = static_cast<int32_t>(forward) - static_cast<int32_t>(strafe) + static_cast<int32_t>(rotate);

	int32_t maxMagnitude = abs(frCmd);
	maxMagnitude = max(maxMagnitude, abs(flCmd));
	maxMagnitude = max(maxMagnitude, abs(brCmd));
	maxMagnitude = max(maxMagnitude, abs(blCmd));
	maxMagnitude = max(maxMagnitude, JOYSTICK_MAX_ABS);

	frCmd = (frCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	flCmd = (flCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	brCmd = (brCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	blCmd = (blCmd * JOYSTICK_MAX_ABS) / maxMagnitude;

	const int32_t driveStepDeltaMax = computeDriveStepDeltaMax();

	const int32_t frDelta = (frCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t flDelta = (flCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t brDelta = (brCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t blDelta = (blCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;

	fr_target += frDelta;
	fl_target += flDelta;
	br_target += brDelta;
	bl_target += blDelta;

	stepper_fr->moveTo(fr_target);
	stepper_fl->moveTo(fl_target);
	stepper_br->moveTo(br_target);
	stepper_bl->moveTo(bl_target);
}
} // namespace robot::movers
