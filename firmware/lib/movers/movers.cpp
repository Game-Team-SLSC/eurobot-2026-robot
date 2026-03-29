#include "movers.h"

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <FastAccelStepper.h>

#include <config.h>
#include <logging.h>

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
bool g_moversReady = false;

constexpr int32_t JOYSTICK_MAX_ABS = 127;

int16_t clampToInt16(int32_t value) {
	if (value > 32767) {
		return 32767;
	}
	if (value < -32768) {
		return -32768;
	}
	return static_cast<int16_t>(value);
}

int32_t getStepperSpeedMilliHz(FastAccelStepper* stepper) {
	if (stepper == nullptr) {
		return 0;
	}
	// realtime=false gives a smoother estimate during accel/decel.
	return stepper->getCurrentSpeedInMilliHz(false);
}

void setAllDriverChipSelectInactive() {
	digitalWrite(robot::config::tmc_bl_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_fl_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_br_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_fr_config.csPin, HIGH);
}

void applyDriverSettings(TMC5160Stepper& drv) {
	drv.begin();
	// drv.toff(4);
	// drv.blank_time(24);
	drv.rms_current(robot::config::motor_rms_current_ma);
	drv.microsteps(robot::config::motor_microsteps);
	drv.en_pwm_mode(true);
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
	robot::logging::info("movers", "init start");

	pinMode(robot::config::tmc_bl_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_br_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_fl_config.csPin, OUTPUT);
    pinMode(robot::config::tmc_fr_config.csPin, OUTPUT);
    
	setAllDriverChipSelectInactive();

    applyDriverSettings(driver_fr);
    applyDriverSettings(driver_fl);
    applyDriverSettings(driver_br);
    applyDriverSettings(driver_bl);
	// Ensure no driver keeps MISO active after config.
	setAllDriverChipSelectInactive();

	engine.init();

	stepper_fr = engine.stepperConnectToPin(robot::config::tmc_fr_config.stepPin);
	stepper_fl = engine.stepperConnectToPin(robot::config::tmc_fl_config.stepPin);
	stepper_br = engine.stepperConnectToPin(robot::config::tmc_br_config.stepPin);
	stepper_bl = engine.stepperConnectToPin(robot::config::tmc_bl_config.stepPin);

	applyStepperSettings(stepper_fr, robot::config::tmc_fr_config);
    applyStepperSettings(stepper_fl, robot::config::tmc_fl_config);
    applyStepperSettings(stepper_br, robot::config::tmc_br_config);
    applyStepperSettings(stepper_bl, robot::config::tmc_bl_config);

	g_moversReady = true;

	return true;
}

void drive(int8_t forward, int8_t strafe, int8_t rotate) {
	if (!g_moversReady || (stepper_fr == nullptr) || (stepper_fl == nullptr) ||
		(stepper_br == nullptr) || (stepper_bl == nullptr)) {
		static uint32_t lastWarnMs = 0;
		const uint32_t nowMs = millis();
		if ((nowMs - lastWarnMs) >= 1000U) {
			lastWarnMs = nowMs;
			robot::logging::warn("movers", "drive called while movers not ready");
		}
		return;
	}

	int32_t frCmd = static_cast<int32_t>(forward) + static_cast<int32_t>(strafe) + static_cast<int32_t>(rotate);
	int32_t flCmd = static_cast<int32_t>(forward) - static_cast<int32_t>(strafe) - static_cast<int32_t>(rotate);
	int32_t brCmd = static_cast<int32_t>(forward) - static_cast<int32_t>(strafe) + static_cast<int32_t>(rotate);
	int32_t blCmd = static_cast<int32_t>(forward) + static_cast<int32_t>(strafe) - static_cast<int32_t>(rotate);

	int32_t maxMagnitude = abs(frCmd);
	maxMagnitude = max(maxMagnitude, abs(flCmd));
	maxMagnitude = max(maxMagnitude, abs(brCmd));
	maxMagnitude = max(maxMagnitude, abs(blCmd));
	maxMagnitude = max(maxMagnitude, JOYSTICK_MAX_ABS);

	frCmd = (frCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	flCmd = (flCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	brCmd = (brCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	blCmd = (blCmd * JOYSTICK_MAX_ABS) / maxMagnitude;

	const int32_t driveStepDeltaMax = 220;

	const int32_t frDelta = (frCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t flDelta = (flCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t brDelta = (brCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;
	const int32_t blDelta = (blCmd * driveStepDeltaMax) / JOYSTICK_MAX_ABS;

	Serial.printf("[movers] drive cmd f=%d s=%d r=%d -> frDelta=%ld flDelta=%ld brDelta=%ld blDelta=%ld\n",
	              static_cast<int>(forward),
	              static_cast<int>(strafe),
	              static_cast<int>(rotate),
	              static_cast<long>(frDelta),
	              static_cast<long>(flDelta),
	              static_cast<long>(brDelta),
	              static_cast<long>(blDelta));

	fr_target += frDelta;
	fl_target += flDelta;
	br_target += brDelta;
	bl_target += blDelta;

	stepper_fr->move(frDelta);
	stepper_fl->move(flDelta);
	stepper_br->move(brDelta);
	stepper_bl->move(blDelta);

	static uint32_t lastDriveLogMs = 0;
	const uint32_t nowMs = millis();
	if ((nowMs - lastDriveLogMs) >= 500U) {
		lastDriveLogMs = nowMs;
		robot::logging::infof("movers", "cmd f=%d s=%d r=%d tgt fr=%ld fl=%ld br=%ld bl=%ld",
		                     static_cast<int>(forward),
		                     static_cast<int>(strafe),
		                     static_cast<int>(rotate),
		                     static_cast<long>(fr_target),
		                     static_cast<long>(fl_target),
		                     static_cast<long>(br_target),
		                     static_cast<long>(bl_target));
	}
}

Vec3 getCurrentVelocity() {
	Vec3 vel{0, 0, 0};

	if (!g_moversReady || (stepper_fr == nullptr) || (stepper_fl == nullptr) ||
		(stepper_br == nullptr) || (stepper_bl == nullptr)) {
		return vel;
	}

	const int32_t frSpeedMilliHz = getStepperSpeedMilliHz(stepper_fr);
	const int32_t flSpeedMilliHz = getStepperSpeedMilliHz(stepper_fl);
	const int32_t brSpeedMilliHz = getStepperSpeedMilliHz(stepper_br);
	const int32_t blSpeedMilliHz = getStepperSpeedMilliHz(stepper_bl);

	// Inverse mixing from wheel speeds to robot axes.
	const int32_t forwardMilliHz = (frSpeedMilliHz + flSpeedMilliHz + brSpeedMilliHz + blSpeedMilliHz) / 4;
	const int32_t strafeMilliHz = (frSpeedMilliHz - flSpeedMilliHz - brSpeedMilliHz + blSpeedMilliHz) / 4;
	const int32_t rotateMilliHz = (frSpeedMilliHz - flSpeedMilliHz + brSpeedMilliHz - blSpeedMilliHz) / 4;

	const int32_t maxWheelMilliHz = static_cast<int32_t>(robot::config::motion_speed_hz) * 1000;
	if (maxWheelMilliHz <= 0) {
		return vel;
	}

	const int32_t forwardNorm = (forwardMilliHz * JOYSTICK_MAX_ABS) / maxWheelMilliHz;
	const int32_t strafeNorm = (strafeMilliHz * JOYSTICK_MAX_ABS) / maxWheelMilliHz;
	const int32_t rotateNorm = (rotateMilliHz * JOYSTICK_MAX_ABS) / maxWheelMilliHz;

	vel.forward = clampToInt16(forwardNorm);
	vel.strafe = clampToInt16(strafeNorm);
	vel.rotate = clampToInt16(rotateNorm);
	Serial.printf("[movers] velocity fwd=%d strafe=%d rot=%d (raw fwd=%ld strafe=%ld rot=%ld)\n",
	              static_cast<int>(vel.forward),
	              static_cast<int>(vel.strafe),
	              static_cast<int>(vel.rotate),
	              static_cast<long>(forwardMilliHz),
	              static_cast<long>(strafeMilliHz),
	              static_cast<long>(rotateMilliHz));
	return vel;
}

} // namespace robot::movers