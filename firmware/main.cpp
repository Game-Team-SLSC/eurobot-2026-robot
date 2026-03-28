#include <RF24.h>
#include "nRF24L01.h"
#include <SPI.h>
#include <TCA9555.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <RemoteData.h>

#define RF_ADDRESS "GT912"
#define RF_CHANNEL 100

using robot::types::RemoteData;

constexpr uint32_t MY_SCK = 12;
constexpr uint32_t MY_MISO = 11;
constexpr uint32_t MY_MOSI = 21;
constexpr uint8_t RADIO_CE_PIN = 48;
constexpr uint8_t RADIO_CSN_PIN = 38;

constexpr uint8_t AXIS_COUNT = 4;
constexpr uint8_t AXIS_FR = 0;
constexpr uint8_t AXIS_FL = 1;
constexpr uint8_t AXIS_BR = 2;
constexpr uint8_t AXIS_BL = 3;

// Update these pin constants for your wiring.
constexpr uint8_t TMC_CS_FR_PIN = 39;
constexpr uint8_t TMC_CS_FL_PIN = 40;
constexpr uint8_t TMC_CS_BR_PIN = 9;
constexpr uint8_t TMC_CS_BL_PIN = 8;

constexpr uint8_t STEP_FR_PIN = 41;
constexpr uint8_t STEP_FL_PIN = 42;
constexpr uint8_t STEP_BR_PIN = 15;
constexpr uint8_t STEP_BL_PIN = 16;

constexpr uint8_t DIR_FR_PIN = 1;
constexpr uint8_t DIR_FL_PIN = 2;
constexpr uint8_t DIR_BR_PIN = 18;
constexpr uint8_t DIR_BL_PIN = 17;

constexpr uint8_t TMC_EN_PIN = 40;
constexpr uint8_t I2C_SDA_PIN = 13;
constexpr uint8_t I2C_SCL_PIN = 14;
constexpr float tmc_rsense = 0.075f;

constexpr uint16_t MOTOR_RMS_CURRENT_mA = 1200;
constexpr uint16_t MOTOR_MICROSTEPS = 8;
constexpr uint8_t motor_irun = 25;
constexpr uint8_t motor_ihold = 25;
constexpr uint8_t motor_iholddelay = 10;
constexpr uint32_t motion_speed_hz = 15000;
constexpr uint32_t motion_accel = 25000;
constexpr uint16_t JOYSTICK_DEADZONE = 3;
constexpr uint16_t JOYSTICK_MAX_ABS = 127;
constexpr uint16_t DRIVE_STEP_DELTA_MAX = 220;
constexpr uint8_t DRIVE_CMD_SLEW_PER_TICK = 32;
constexpr uint8_t DRIVE_BRAKE_SLEW_PER_TICK = 48;
constexpr uint8_t DRIVE_NEUTRAL_SLEW_PER_TICK = 16;
constexpr uint16_t STRAFE_GAIN_PCT = 115;
constexpr uint16_t ROTATE_GAIN_PCT = 130;
constexpr uint16_t HIGH_SPEED_STRAFE_MIN_PCT = 62;
constexpr uint16_t HIGH_SPEED_ROTATE_MIN_PCT = 72;
constexpr uint16_t HIGH_SPEED_TRACTION_MIN_PCT = 82;
constexpr uint32_t RADIO_TIMEOUT_MS = 400;
constexpr uint32_t RADIO_BRAKE_TO_HOLD_MS = 80;
constexpr uint32_t NEUTRAL_HOLD_DELAY_MS = 60;
constexpr int32_t TRAVEL_STEPS = 20000 / 3;
constexpr int32_t ROTATE_360_STEPS = 12000;
constexpr uint32_t SHARED_SPI_HZ = 2000000;
constexpr uint32_t RF24_SPI_HZ = 2000000;

constexpr uint8_t TMC_CS_PINS[AXIS_COUNT] = {TMC_CS_FR_PIN, TMC_CS_FL_PIN, TMC_CS_BR_PIN, TMC_CS_BL_PIN};
constexpr uint8_t STEP_PINS[AXIS_COUNT] = {STEP_FR_PIN, STEP_FL_PIN, STEP_BR_PIN, STEP_BL_PIN};
constexpr uint8_t DIR_PINS[AXIS_COUNT] = {DIR_FR_PIN, DIR_FL_PIN, DIR_BR_PIN, DIR_BL_PIN};
constexpr bool DIR_HIGH_COUNTS_UP[AXIS_COUNT] = {false, true, true, true};
constexpr const char *AXIS_NAMES[AXIS_COUNT] = {"FR", "FL", "BR", "BL"};

TCA9555 tca9555(0x27, &Wire); // I2C address of the TCA9555 I/O expander

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *steppers[AXIS_COUNT] = {nullptr, nullptr, nullptr, nullptr};
TMC5160Stepper driverFR(TMC_CS_FR_PIN, MY_MOSI, MY_MISO, MY_SCK);
TMC5160Stepper driverFL(TMC_CS_FL_PIN, MY_MOSI, MY_MISO, MY_SCK);
TMC5160Stepper driverBR(TMC_CS_BR_PIN, MY_MOSI, MY_MISO, MY_SCK);
TMC5160Stepper driverBL(TMC_CS_BL_PIN, MY_MOSI, MY_MISO, MY_SCK);
TMC5160Stepper *drivers[AXIS_COUNT] = {&driverFR, &driverFL, &driverBR, &driverBL};

SPIClass hspi(HSPI);

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN, RF24_SPI_HZ);

enum DemoPhase : uint8_t {
  DEMO_FWD_1,
  DEMO_ROT_360,
  DEMO_FWD_2,
  DEMO_ROT_180,
  DEMO_BACK_TO_ORIGIN,
  DEMO_ROT_TO_ORIGIN,
  DEMO_DONE,
};

DemoPhase demoPhase = DEMO_FWD_1;
bool demoStarted = false;

bool driverDetected[AXIS_COUNT] = {false, false, false, false};
bool driverPreviouslyDetected[AXIS_COUNT] = {false, false, false, false};
bool tcaReady = false;
int32_t axisTargets[AXIS_COUNT] = {0, 0, 0, 0};
uint32_t lastRadioPacketMs = 0;
uint32_t lastDriveDebugMs = 0;
bool radioReady = false;
int16_t smoothedForward = 0;
int16_t smoothedStrafe = 0;
int16_t smoothedRotate = 0;
uint32_t lastMotionCommandMs = 0;

bool allSteppersConfigured();

static void setAllTmcChipSelectInactive() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    pinMode(TMC_CS_PINS[axis], OUTPUT);
    digitalWrite(TMC_CS_PINS[axis], HIGH);
  }
}

bool configureTca() {
  if (!tca9555.begin()) {
    Serial.printf("[TCA] begin failed, err=0x%02X\n", tca9555.lastError());
    return false;
  }
  if (!tca9555.isConnected()) {
    Serial.printf("[TCA] not connected @0x%02X, err=0x%02X\n", tca9555.getAddress(), tca9555.lastError());
    return false;
  }

  // Pin 7 means P07 in this library numbering (0..15).
  const bool modeOk = tca9555.pinMode1(7, OUTPUT);
  const bool writeOk = tca9555.write1(7, LOW);
  const uint8_t readBack = tca9555.read1(7);
  const int tcaErr = tca9555.lastError();

  Serial.printf("[TCA] pinMode=%u write=%u readBack=%u err=0x%02X\n",
                static_cast<unsigned int>(modeOk),
                static_cast<unsigned int>(writeOk),
                static_cast<unsigned int>(readBack),
                static_cast<unsigned int>(tcaErr));

  return modeOk && writeOk && (tcaErr == TCA9555_OK);
}

static void printRemoteData(const RemoteData &data) {
  Serial.print("Received: joystickLeft=(");
  Serial.print(data.joystickLeft.x);
  Serial.print(",");
  Serial.print(data.joystickLeft.y);
  Serial.print(") joystickRight=(");
  Serial.print(data.joystickRight.x);
  Serial.print(",");
  Serial.print(data.joystickRight.y);
  Serial.print(") slider=");
  Serial.print(data.slider);
  Serial.print(" score=");
  Serial.print(data.score);
  Serial.print(" buttons=[");

  for (uint8_t i = 0; i < 15; ++i) {
    Serial.print(data.buttons[i] ? 1 : 0);
    if (i < 14) {
      Serial.print(",");
    }
  }

  Serial.println("]");
}

static uint64_t rfAddressToUint64(const char *addr) {
  uint64_t packed = 0;
  for (uint8_t i = 0; i < 5 && addr[i] != '\0'; ++i) {
    packed |= (static_cast<uint64_t>(static_cast<uint8_t>(addr[i])) << (8 * i));
  }
  return packed;
}

static int16_t normalizeJoystickAxis(byte value, bool invert) {
  uint8_t snappedValue = value;
  if ((snappedValue >= 125) && (snappedValue <= 129)) {
    snappedValue = 127;
  }

  int16_t centered = static_cast<int16_t>(snappedValue) - 127;
  if (invert) {
    centered = -centered;
  }
  return constrain(centered, -static_cast<int16_t>(JOYSTICK_MAX_ABS), static_cast<int16_t>(JOYSTICK_MAX_ABS));
}

static int16_t applyPercentGainAndClamp(int16_t value, uint16_t gainPct) {
  int32_t scaled = (static_cast<int32_t>(value) * static_cast<int32_t>(gainPct)) / 100;
  return static_cast<int16_t>(constrain(scaled,
                                        -static_cast<int32_t>(JOYSTICK_MAX_ABS),
                                        static_cast<int32_t>(JOYSTICK_MAX_ABS)));
}

static int16_t applyAggressiveResponseCurve(int16_t value) {
  return value;
}

static int16_t applySlewStep(int16_t current, int16_t target, int16_t maxStep) {
  if (target > current) {
    return static_cast<int16_t>(min<int32_t>(current + maxStep, target));
  }
  if (target < current) {
    return static_cast<int16_t>(max<int32_t>(current - maxStep, target));
  }
  return current;
}

static int16_t applyDriveSlew(int16_t current, int16_t target) {
  return target;
}

static void stopAllMotionHold() {
  if (!allSteppersConfigured()) {
    return;
  }

  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    steppers[axis]->forceStopAndNewPosition(axisTargets[axis]);
  }
}

static void applyMecanumMix(int16_t forward, int16_t strafe, int16_t rotate) {
  if (!allSteppersConfigured()) {
    static uint32_t lastWarnMs = 0;
    const uint32_t now = millis();
    if ((now - lastWarnMs) > 1000) {
      lastWarnMs = now;
      Serial.println("[DRIVE] steppers not configured, command ignored");
    }
    return;
  }

  const int32_t strafeEff = static_cast<int32_t>(strafe);
  const int32_t rotateEff = static_cast<int32_t>(rotate);

  int32_t frCmd = static_cast<int32_t>(forward) - strafeEff - rotateEff;
  int32_t flCmd = static_cast<int32_t>(forward) + strafeEff + rotateEff;
  int32_t brCmd = static_cast<int32_t>(forward) + strafeEff - rotateEff;
  int32_t blCmd = static_cast<int32_t>(forward) - strafeEff + rotateEff;

  int32_t maxMagnitude = abs(frCmd);
  maxMagnitude = max(maxMagnitude, abs(flCmd));
  maxMagnitude = max(maxMagnitude, abs(brCmd));
  maxMagnitude = max(maxMagnitude, abs(blCmd));
  maxMagnitude = max(maxMagnitude, static_cast<int32_t>(JOYSTICK_MAX_ABS));

  frCmd = (frCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
  flCmd = (flCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
  brCmd = (brCmd * JOYSTICK_MAX_ABS) / maxMagnitude;
  blCmd = (blCmd * JOYSTICK_MAX_ABS) / maxMagnitude;

  const int32_t deltaMax = static_cast<int32_t>(DRIVE_STEP_DELTA_MAX);

  const int32_t frDelta = (frCmd * deltaMax) / JOYSTICK_MAX_ABS;
  const int32_t flDelta = (flCmd * deltaMax) / JOYSTICK_MAX_ABS;
  const int32_t brDelta = (brCmd * deltaMax) / JOYSTICK_MAX_ABS;
  const int32_t blDelta = (blCmd * deltaMax) / JOYSTICK_MAX_ABS;

  axisTargets[AXIS_FR] += frDelta;
  axisTargets[AXIS_FL] += flDelta;
  axisTargets[AXIS_BR] += brDelta;
  axisTargets[AXIS_BL] += blDelta;

  steppers[AXIS_FR]->moveTo(axisTargets[AXIS_FR]);
  steppers[AXIS_FL]->moveTo(axisTargets[AXIS_FL]);
  steppers[AXIS_BR]->moveTo(axisTargets[AXIS_BR]);
  steppers[AXIS_BL]->moveTo(axisTargets[AXIS_BL]);

  const uint32_t now = millis();
  if (((forward != 0) || (strafe != 0) || (rotate != 0)) && ((now - lastDriveDebugMs) > 250)) {
    lastDriveDebugMs = now;
    Serial.printf("[DRIVE] fwd=%d str=%d rot=%d dFR=%ld dFL=%ld dBR=%ld dBL=%ld run=%u%u%u%u\n",
                  static_cast<int>(forward),
                  static_cast<int>(strafe),
                  static_cast<int>(rotate),
                  static_cast<long>(frDelta),
                  static_cast<long>(flDelta),
                  static_cast<long>(brDelta),
                  static_cast<long>(blDelta),
                  static_cast<unsigned int>(steppers[AXIS_FR]->isRunning()),
                  static_cast<unsigned int>(steppers[AXIS_FL]->isRunning()),
                  static_cast<unsigned int>(steppers[AXIS_BR]->isRunning()),
                  static_cast<unsigned int>(steppers[AXIS_BL]->isRunning()));
  }
}

static void controlMotorsFromRemote(const RemoteData &data) {
  const int16_t forwardTarget = applyAggressiveResponseCurve(
      normalizeJoystickAxis(data.joystickLeft.y, true));
  const int16_t strafeTarget = applyPercentGainAndClamp(
      applyAggressiveResponseCurve(normalizeJoystickAxis(data.joystickLeft.x, false)),
      STRAFE_GAIN_PCT);
  const int16_t rotateTarget = applyPercentGainAndClamp(
      applyAggressiveResponseCurve(normalizeJoystickAxis(data.joystickRight.x, false)),
      ROTATE_GAIN_PCT);

  smoothedForward = applyDriveSlew(smoothedForward, forwardTarget);
  smoothedStrafe = applyDriveSlew(smoothedStrafe, strafeTarget);
  smoothedRotate = applyDriveSlew(smoothedRotate, rotateTarget);

  if ((smoothedForward != 0) || (smoothedStrafe != 0) || (smoothedRotate != 0)) {
    lastMotionCommandMs = millis();
    applyMecanumMix(smoothedForward, smoothedStrafe, smoothedRotate);
    return;
  }

  applyMecanumMix(0, 0, 0);
  if ((millis() - lastMotionCommandMs) > NEUTRAL_HOLD_DELAY_MS) {
    stopAllMotionHold();
  }
}

void applyDriverSettings(TMC5160Stepper &drv) {
  drv.begin();
  drv.toff(4);
  drv.blank_time(24);
  drv.rms_current(MOTOR_RMS_CURRENT_mA);
  drv.microsteps(MOTOR_MICROSTEPS);
  drv.ihold(motor_ihold);
  drv.irun(motor_irun);
  drv.iholddelay(motor_iholddelay);
}

void configureSteppers() {
  engine.init();

  if ((TMC_EN_PIN == TMC_CS_FR_PIN) || (TMC_EN_PIN == TMC_CS_FL_PIN) ||
      (TMC_EN_PIN == TMC_CS_BR_PIN) || (TMC_EN_PIN == TMC_CS_BL_PIN)) {
    Serial.printf("[BOOT] WARNING: TMC_EN_PIN (%u) conflicts with a CS pin, keeping EN forced LOW\n",
                  static_cast<unsigned int>(TMC_EN_PIN));
  }

  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    steppers[axis] = engine.stepperConnectToPin(STEP_PINS[axis]);
    if (steppers[axis] == nullptr) {
      Serial.printf("[BOOT][%s] FastAccelStepper init failed\n", AXIS_NAMES[axis]);
      continue;
    }

    steppers[axis]->setDirectionPin(DIR_PINS[axis], DIR_HIGH_COUNTS_UP[axis]);
    steppers[axis]->setSpeedInHz(motion_speed_hz);
    steppers[axis]->setAcceleration(motion_accel);
    steppers[axis]->forceStopAndNewPosition(0);
    Serial.printf("[BOOT][%s] configured, hold torque active\n", AXIS_NAMES[axis]);
  }
}

void queueRelativeMove(int32_t fr, int32_t fl, int32_t br, int32_t bl, const char *label) {
  const int32_t deltas[AXIS_COUNT] = {fr, fl, br, bl};
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    axisTargets[axis] += deltas[axis];
    steppers[axis]->moveTo(axisTargets[axis]);
  }
  Serial.printf("[DEMO] %s\n", label);
}
bool allSteppersConfigured() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    if (steppers[axis] == nullptr) {
      return false;
    }
  }
  return true;
}

bool allSteppersIdle() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    if ((steppers[axis] != nullptr) && steppers[axis]->isRunning()) {
      return false;
    }
  }
  return true;
}

void runDemoSequence() {
  if (!allSteppersConfigured()) {
    return;
  }

  if (!demoStarted) {
    demoStarted = true;
    queueRelativeMove(TRAVEL_STEPS, TRAVEL_STEPS, TRAVEL_STEPS, TRAVEL_STEPS, "forward #1");
    return;
  }

  if (!allSteppersIdle()) {
    return;
  }

  switch (demoPhase) {
    case DEMO_FWD_1:
      demoPhase = DEMO_ROT_360;
      queueRelativeMove(-ROTATE_360_STEPS, ROTATE_360_STEPS, -ROTATE_360_STEPS, ROTATE_360_STEPS, "rotate 360");
      break;

    case DEMO_ROT_360:
      demoPhase = DEMO_FWD_2;
      queueRelativeMove(TRAVEL_STEPS, TRAVEL_STEPS, TRAVEL_STEPS, TRAVEL_STEPS, "forward #2");
      break;

    case DEMO_FWD_2:
      demoPhase = DEMO_ROT_180;
      queueRelativeMove(-(ROTATE_360_STEPS / 2), (ROTATE_360_STEPS / 2), -(ROTATE_360_STEPS / 2), (ROTATE_360_STEPS / 2), "rotate 180");
      break;

    case DEMO_ROT_180:
      demoPhase = DEMO_BACK_TO_ORIGIN;
      queueRelativeMove(2 * TRAVEL_STEPS, 2 * TRAVEL_STEPS, 2 * TRAVEL_STEPS, 2 * TRAVEL_STEPS, "go back to origin position");
      break;

    case DEMO_BACK_TO_ORIGIN:
      demoPhase = DEMO_ROT_TO_ORIGIN;
      queueRelativeMove(-(ROTATE_360_STEPS / 2), (ROTATE_360_STEPS / 2), -(ROTATE_360_STEPS / 2), (ROTATE_360_STEPS / 2), "rotate back to origin heading");
      break;

    case DEMO_ROT_TO_ORIGIN:
      demoPhase = DEMO_DONE;
      Serial.println("[DEMO] done");
      break;

    case DEMO_DONE:
      break;
  }
}

void configureDrivers() {
  setAllTmcChipSelectInactive();
  for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
    applyDriverSettings(*drivers[axis]);
  }
  setAllTmcChipSelectInactive();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  tcaReady = configureTca();
  Serial.printf("[TCA] ready=%s\n", tcaReady ? "YES" : "NO");

  // Keep RF24 deselected while TMC drivers are configured.
  pinMode(RADIO_CSN_PIN, OUTPUT);
  digitalWrite(RADIO_CSN_PIN, HIGH);
  pinMode(RADIO_CE_PIN, OUTPUT);
  digitalWrite(RADIO_CE_PIN, LOW);

  setAllTmcChipSelectInactive();

  configureDrivers();
  configureSteppers();

  // Initialize RF24 last on HSPI once TMC setup is complete.
  hspi.begin(MY_SCK, MY_MISO, MY_MOSI);

  hspi.setFrequency(RF24_SPI_HZ);
  radioReady = radio.begin(&hspi);
  if (!radioReady) {
    Serial.println("[RADIO] begin failed, continuing with forced config");
    Serial.print("[RADIO] isChipConnected: ");
    Serial.println(radio.isChipConnected());
  }

  radio.openReadingPipe(1, rfAddressToUint64(RF_ADDRESS));
  radio.setChannel(static_cast<uint8_t>(RF_CHANNEL));
  radio.setDataRate(RF24_250KBPS);

  radio.startListening();
  Serial.printf("[RADIO] ready=%u channel=%u spi=%luHz\n",
                static_cast<unsigned int>(radioReady),
                static_cast<unsigned int>(RF_CHANNEL),
                static_cast<unsigned long>(RF24_SPI_HZ));
  
  radio.printDetails();
}

void loop() {
  RemoteData receivedData;
  const uint32_t nowMs = millis();
  static bool timeoutHoldApplied = false;

  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    lastRadioPacketMs = nowMs;
    timeoutHoldApplied = false;
    printRemoteData(receivedData);
    controlMotorsFromRemote(receivedData);
  } else if ((nowMs - lastRadioPacketMs) > RADIO_TIMEOUT_MS) {
    smoothedForward = 0;
    smoothedStrafe = 0;
    smoothedRotate = 0;
    applyMecanumMix(0, 0, 0);

    if (!timeoutHoldApplied &&
        ((nowMs - lastRadioPacketMs) > (RADIO_TIMEOUT_MS + RADIO_BRAKE_TO_HOLD_MS)) &&
        allSteppersIdle()) {
      timeoutHoldApplied = true;
      stopAllMotionHold();
      Serial.println("[DRIVE] radio timeout -> brake then hold");
    }
  }

  delay(10);
}