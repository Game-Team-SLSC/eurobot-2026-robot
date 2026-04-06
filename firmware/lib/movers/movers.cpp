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
bool g_moversReady = false;

constexpr int32_t JOYSTICK_MAX_ABS = 127;

constexpr uint8_t TMC_TOFF = 4;
constexpr uint8_t TMC_BLANK_TIME = 24;
constexpr uint8_t TMC_IHOLD_PCT = 5;
constexpr uint8_t TMC_IRUN_PCT = 100;
constexpr uint8_t TMC_IHOLDDELAY = 2;

int16_t clampToInt16(int32_t value) {
	if (value > 32767) {
		return 32767;
	}
	if (value < -32768) {
		return -32768;
	}
	return static_cast<int16_t>(value);
}

int32_t targetAxisMmToSteps(int16_t targetMm) {
	const float targetMeters = static_cast<float>(targetMm) * robot::config::mm_to_m;
	return static_cast<int32_t>(lroundf(targetMeters * robot::config::movers_steps_per_meter));
}

int32_t getStepperSpeedMilliHz(FastAccelStepper* stepper) {
	if (stepper == nullptr) {
		return 0;
	}
	// realtime=false gives a smoother estimate during accel/decel.
	return stepper->getCurrentSpeedInMilliHz(false);
}

void applyWheelCommand(FastAccelStepper* stepper, int32_t cmd) {
	if (stepper == nullptr) {
		return;
	}

	const int32_t signedSpeedHz =
		(static_cast<int32_t>(robot::config::motion_speed_hz) * cmd) / JOYSTICK_MAX_ABS;
	uint32_t speedHz = static_cast<uint32_t>((signedSpeedHz < 0) ? -signedSpeedHz : signedSpeedHz);
	stepper->setSpeedInHz(speedHz);

	if (cmd == 0) {
		stepper->stopMove();
	} else

	if (cmd > 0) {
		stepper->runForward();
	} else {
		stepper->runBackward();
	}

	stepper->applySpeedAcceleration();
}

void prepareStepperForPositionMove(FastAccelStepper* stepper) {
	if (stepper == nullptr) {
		return;
	}

	// moveTo() relies on current speed/acceleration settings.
	// drive(0) can leave speed at 0 Hz, so restore nominal motion settings.
	stepper->setSpeedInHz(robot::config::motion_speed_hz);
	stepper->setAcceleration(robot::config::motion_accel);
	stepper->applySpeedAcceleration();
}

void setAllDriverChipSelectInactive() {
	digitalWrite(robot::config::tmc_bl_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_fl_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_br_config.csPin, HIGH);
	digitalWrite(robot::config::tmc_fr_config.csPin, HIGH);
}

void applyDriverSettings(TMC5160Stepper& drv) {
	drv.begin();
	drv.toff(TMC_TOFF);
	drv.blank_time(TMC_BLANK_TIME);
	drv.rms_current(robot::config::motor_rms_current_ma);
	drv.microsteps(robot::config::motor_microsteps);
	drv.ihold((31U * TMC_IHOLD_PCT) / 100U);
	drv.irun(31U);
	drv.iholddelay(TMC_IHOLDDELAY);
	//drv.en_pwm_mode(true);
}

void applyStepperSettings(FastAccelStepper* stepper, const robot::config::TMCConfig& axisConfig) {
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

void drive(MotionCommand& cmd) {
	if (!g_moversReady || (stepper_fr == nullptr) || (stepper_fl == nullptr) ||
		(stepper_br == nullptr) || (stepper_bl == nullptr)) {
		return;
	}

	int32_t frCmd = static_cast<int32_t>(cmd.forward) + static_cast<int32_t>(cmd.strafe) + static_cast<int32_t>(cmd.rotate);
	int32_t flCmd = static_cast<int32_t>(cmd.forward) - static_cast<int32_t>(cmd.strafe) - static_cast<int32_t>(cmd.rotate);
	int32_t brCmd = static_cast<int32_t>(cmd.forward) - static_cast<int32_t>(cmd.strafe) + static_cast<int32_t>(cmd.rotate);
	int32_t blCmd = static_cast<int32_t>(cmd.forward) + static_cast<int32_t>(cmd.strafe) - static_cast<int32_t>(cmd.rotate);

	int32_t maxMagnitude = abs(frCmd);
	maxMagnitude = max(maxMagnitude, abs(flCmd));
	maxMagnitude = max(maxMagnitude, abs(brCmd));
	maxMagnitude = max(maxMagnitude, abs(blCmd));
	maxMagnitude = max(maxMagnitude, JOYSTICK_MAX_ABS);

	frCmd = (frCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	flCmd = (flCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	brCmd = (brCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
	blCmd = (blCmd * JOYSTICK_MAX_ABS) / maxMagnitude;

	applyWheelCommand(stepper_fr, frCmd);
	applyWheelCommand(stepper_fl, flCmd);
	applyWheelCommand(stepper_br, brCmd);
	applyWheelCommand(stepper_bl, blCmd);
}

void goToTarget(const MotionCommand& cmd) {
	if (!g_moversReady || (stepper_fr == nullptr) || (stepper_fl == nullptr) ||
		(stepper_br == nullptr) || (stepper_bl == nullptr)) {
		return;
	}

	const int32_t forwardTargetSteps = targetAxisMmToSteps(-cmd.target.forward);
	const int32_t strafeTargetSteps = targetAxisMmToSteps(cmd.target.strafe);
	const int32_t rotateTargetSteps = targetAxisMmToSteps(cmd.target.rotate);

	fr_target = forwardTargetSteps + strafeTargetSteps + rotateTargetSteps;
	fl_target = forwardTargetSteps - strafeTargetSteps - rotateTargetSteps;
	br_target = forwardTargetSteps - strafeTargetSteps + rotateTargetSteps;
	bl_target = forwardTargetSteps + strafeTargetSteps - rotateTargetSteps;

	prepareStepperForPositionMove(stepper_fr);
	prepareStepperForPositionMove(stepper_fl);
	prepareStepperForPositionMove(stepper_br);
	prepareStepperForPositionMove(stepper_bl);

	const int32_t frCurrent = stepper_fr->getCurrentPosition();
	const int32_t flCurrent = stepper_fl->getCurrentPosition();
	const int32_t brCurrent = stepper_br->getCurrentPosition();
	const int32_t blCurrent = stepper_bl->getCurrentPosition();

	const int32_t frAbsTarget = frCurrent + fr_target;
	const int32_t flAbsTarget = flCurrent + fl_target;
	const int32_t brAbsTarget = brCurrent + br_target;
	const int32_t blAbsTarget = blCurrent + bl_target;

	stepper_fr->moveTo(frAbsTarget);
	stepper_fl->moveTo(flAbsTarget);
	stepper_br->moveTo(brAbsTarget);
	stepper_bl->moveTo(blAbsTarget);
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
	return vel;
}

} // namespace robot::movers