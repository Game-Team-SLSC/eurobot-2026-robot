#include "arm_system.h"

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <TCA9548.h>
#include <TCA9555.h>
#include <Wire.h>

#include "actuator_map.h"
#include "arm_calibration_store.h"
#include "config.h"
#include "hardware_pins.h"

namespace robot::arm_system {

namespace {

TwoWire I2C_SENSORS = TwoWire(0);
TwoWire I2C_ACTUATORS = TwoWire(1);

Adafruit_PWMServoDriver PCA0(config::PCA9685_ADDR_0, I2C_ACTUATORS);
Adafruit_PWMServoDriver PCA1(config::PCA9685_ADDR_1, I2C_ACTUATORS);
Adafruit_PWMServoDriver PCA2(config::PCA9685_ADDR_2, I2C_ACTUATORS);
TCA9548 COLOR_MUX(config::TCA9548_COLOR_ADDR, &I2C_SENSORS);
TCA9555 PUMP_IO(config::TCA9555_PUMP_ADDR, &I2C_ACTUATORS);
arm_calibration::CalibrationStore CAL_STORE;
arm_calibration::ArmCalibrationProfile CAL[2][4];

struct PoseDependency {
  ArmPose targetPose;
  StockIslandState requiredStock;
  FlipperState requiredFlipper;
};

constexpr PoseDependency DEPENDENCIES[] = {
    {ArmPose::STORAGE_ENTRY, StockIslandState::OPEN, FlipperState::STRAIGHT_UNLOCKED},
    {ArmPose::STORAGE_END, StockIslandState::OPEN, FlipperState::STRAIGHT_UNLOCKED},
    {ArmPose::FLIP, StockIslandState::LOCKED, FlipperState::FOLD_UNLOCKED},
};

bool InitTcs34725() {
  // Enable power and RGBC.
  I2C_SENSORS.beginTransmission(config::COLOR_SENSOR_ADDR);
  I2C_SENSORS.write(static_cast<uint8_t>(0x80));
  I2C_SENSORS.write(static_cast<uint8_t>(0x03));
  if (I2C_SENSORS.endTransmission() != 0) {
    return false;
  }
  // Integration time.
  I2C_SENSORS.beginTransmission(config::COLOR_SENSOR_ADDR);
  I2C_SENSORS.write(static_cast<uint8_t>(0x81));
  I2C_SENSORS.write(static_cast<uint8_t>(0xEB));
  I2C_SENSORS.endTransmission();
  // Gain = 4x.
  I2C_SENSORS.beginTransmission(config::COLOR_SENSOR_ADDR);
  I2C_SENSORS.write(static_cast<uint8_t>(0x8F));
  I2C_SENSORS.write(static_cast<uint8_t>(0x01));
  I2C_SENSORS.endTransmission();
  return true;
}

bool I2CRead(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t len) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) {
    return false;
  }
  uint8_t got = bus.requestFrom(static_cast<int>(addr), static_cast<int>(len));
  if (got != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; ++i) {
    out[i] = static_cast<uint8_t>(bus.read());
  }
  return true;
}

void PumpWriteMask(uint16_t mask) {
  PUMP_IO.write16(mask);
}

void SelectColorMuxChannel(uint8_t channel) {
  COLOR_MUX.selectChannel(channel);
}

ColorSample ReadColorSample() {
  ColorSample s;
  uint8_t raw[8] = {0};
  // TCS34725 CDATAL register block starts at 0x14 with command bit 0x80.
  if (!I2CRead(I2C_SENSORS, config::COLOR_SENSOR_ADDR, 0x94, raw, 8)) {
    return s;
  }
  s.clear = static_cast<uint16_t>(raw[0] | (raw[1] << 8));
  s.red = static_cast<uint16_t>(raw[2] | (raw[3] << 8));
  s.green = static_cast<uint16_t>(raw[4] | (raw[5] << 8));
  s.blue = static_cast<uint16_t>(raw[6] | (raw[7] << 8));
  s.valid = true;

  const float blueRatio = static_cast<float>(s.blue) / static_cast<float>(s.red + 1U);
  const float yellowRatio = static_cast<float>(s.red + s.green) / static_cast<float>(s.blue + 1U);
  if (blueRatio >= config::COLOR_BLUE_RATIO_MIN) {
    s.color = GameColor::BLUE;
  } else if (yellowRatio >= config::COLOR_YELLOW_RATIO_MIN) {
    s.color = GameColor::YELLOW;
  } else {
    s.color = GameColor::UNKNOWN;
  }

  return s;
}

bool SelectPcaChannel(uint8_t pcaIndex, uint8_t channel, uint16_t pulse) {
  switch (pcaIndex) {
    case 0:
      PCA0.setPWM(channel, 0, pulse);
      return true;
    case 1:
      PCA1.setPWM(channel, 0, pulse);
      return true;
    case 2:
      PCA2.setPWM(channel, 0, pulse);
      return true;
    default:
      return false;
  }
}

void ApplyStockIslandServo(Side side, StockIslandState state) {
  const uint8_t s = (side == Side::LEFT) ? 0 : 1;
  const auto& ch = actuator_map::STOCK_ISLAND_SERVO[s];
  const uint16_t pulse = (state == StockIslandState::OPEN)
                             ? config::STOCK_ISLAND_OPEN_PULSE
                             : config::STOCK_ISLAND_LOCKED_PULSE;
  SelectPcaChannel(ch.pcaIndex, ch.channel, pulse);
}

void ApplyFlipperServos(Side side, FlipperState state) {
  const uint8_t s = (side == Side::LEFT) ? 0 : 1;
  const auto& base = actuator_map::FLIP_BASE_SERVO[s];
  const auto& lock = actuator_map::FLIP_LOCK_SERVO[s];
  const bool fold = (state == FlipperState::FOLD_LOCKED || state == FlipperState::FOLD_UNLOCKED);
  const bool locked = (state == FlipperState::FOLD_LOCKED || state == FlipperState::STRAIGHT_LOCKED);
  SelectPcaChannel(base.pcaIndex, base.channel, fold ? config::FLIP_FOLD_PULSE : config::FLIP_STRAIGHT_PULSE);
  SelectPcaChannel(lock.pcaIndex, lock.channel, locked ? config::FLIP_LOCKED_PULSE : config::FLIP_UNLOCKED_PULSE);
}

}  // namespace

bool ArmSystem::begin() {
  if (pins::i2c::SENSORS_SDA >= 0 && pins::i2c::SENSORS_SCL >= 0) {
    I2C_SENSORS.begin(pins::i2c::SENSORS_SDA, pins::i2c::SENSORS_SCL, 400000U);
  }
  if (pins::i2c::ACTUATORS_SDA >= 0 && pins::i2c::ACTUATORS_SCL >= 0) {
    I2C_ACTUATORS.begin(pins::i2c::ACTUATORS_SDA, pins::i2c::ACTUATORS_SCL, 400000U);
  }

  PCA0.begin();
  PCA1.begin();
  PCA2.begin();
  PCA0.setPWMFreq(50);
  PCA1.setPWMFreq(50);
  PCA2.setPWMFreq(50);

  COLOR_MUX.begin();

  PUMP_IO.begin();
  for (uint8_t pin = 0; pin < 16; ++pin) {
    PUMP_IO.pinMode1(pin, OUTPUT);
  }
  PumpWriteMask(0x0000);

  CAL_STORE.begin();
  for (uint8_t side = 0; side < 2; ++side) {
    for (uint8_t arm = 0; arm < 4; ++arm) {
      CAL_STORE.load(side, arm, &CAL[side][arm]);
    }
  }

  for (uint8_t ch = 0; ch < ColorSensorsSnapshot::COUNT; ++ch) {
    SelectColorMuxChannel(ch);
    InitTcs34725();
  }

  initialized_ = true;
  return true;
}

void ArmSystem::executeImmediatePose(Side side, ArmPose pose) {
  const uint8_t s = (side == Side::LEFT) ? 0 : 1;
  const uint8_t poseIdx = static_cast<uint8_t>(pose);
  const auto& cfg = CAL[s][0].pose[poseIdx];

  const auto& ch0 = actuator_map::ARM_SERVO[s][0][0];
  const auto& ch1 = actuator_map::ARM_SERVO[s][0][1];
  const auto& ch2 = actuator_map::ARM_SERVO[s][0][2];
  SelectPcaChannel(ch0.pcaIndex, ch0.channel, cfg.axis0);
  SelectPcaChannel(ch1.pcaIndex, ch1.channel, cfg.axis1);
  SelectPcaChannel(ch2.pcaIndex, ch2.channel, cfg.axis2);

  if (side == Side::LEFT) {
    leftState_.armPose = pose;
  } else {
    rightState_.armPose = pose;
  }
}

void ArmSystem::planArmPath(Side side, ArmPose target) {
  // Explicit and extensible dependency rules.
  SideActuatorState& st = (side == Side::LEFT) ? leftState_ : rightState_;

  for (const auto& dep : DEPENDENCIES) {
    if (dep.targetPose != target) {
      continue;
    }
    if (st.stockIsland != dep.requiredStock) {
      st.stockIsland = dep.requiredStock;
      ApplyStockIslandServo(side, st.stockIsland);
    }
    if (st.flipper != dep.requiredFlipper) {
      st.flipper = dep.requiredFlipper;
      ApplyFlipperServos(side, st.flipper);
    }
    if (side == Side::LEFT) {
      pendingLeftPose_ = target;
      pendingLeftAtMs_ = millis() + 120;
    } else {
      pendingRightPose_ = target;
      pendingRightAtMs_ = millis() + 120;
    }
    return;
  }

  executeImmediatePose(side, target);
}

void ArmSystem::update(uint32_t nowMs) {
  if (!initialized_) {
    return;
  }

  if (pendingLeftAtMs_ != 0 && nowMs >= pendingLeftAtMs_) {
    executeImmediatePose(Side::LEFT, pendingLeftPose_);
    pendingLeftAtMs_ = 0;
  }
  if (pendingRightAtMs_ != 0 && nowMs >= pendingRightAtMs_) {
    executeImmediatePose(Side::RIGHT, pendingRightPose_);
    pendingRightAtMs_ = 0;
  }
}

bool ArmSystem::shouldFlipForSide(Side side, const JoystickCommand& command) const {
  if (!command.requestColorBasedFlip) {
    return false;
  }

  const uint8_t index = (side == Side::LEFT) ? 0 : 4;
  const ColorSample& sample = lastColors_.samples[index];
  if (!sample.valid || sample.color == GameColor::UNKNOWN) {
    return false;
  }

  // Flip when detected color does not match the target color requested by operator.
  return sample.color != command.targetColor;
}

void ArmSystem::applyAuxOutputs(const JoystickCommand& command) {
  const uint16_t ledPulse = static_cast<uint16_t>((command.slider * 4095U) / 255U);
  const uint16_t fanPulse = (command.buttonsMask & (1U << 12)) ? 4095U :
                            ((command.buttonsMask & (1U << 13)) ? 2048U : 0U);

  SelectPcaChannel(actuator_map::LEDS.pcaIndex, actuator_map::LEDS.channel, ledPulse);
  SelectPcaChannel(actuator_map::FANS.pcaIndex, actuator_map::FANS.channel, fanPulse);
}

void ArmSystem::applyCommand(const JoystickCommand& command) {
  if (!initialized_) {
    return;
  }

  // Minimal action mapping: move two representative servos and toggle pumps.
  uint16_t leftTargetPulse = config::SERVO_PULSE_MID;
  uint16_t rightTargetPulse = config::SERVO_PULSE_MID;

  switch (command.leftIntent) {
    case ActionIntent::GRAB:
      leftTargetPulse = config::SERVO_PULSE_MIN;
      pumpMask_ |= 0x0001;
      break;
    case ActionIntent::STORE:
      leftTargetPulse = config::SERVO_PULSE_MAX;
      pumpMask_ &= static_cast<uint16_t>(~0x0001);
      break;
    case ActionIntent::FLIP:
      planArmPath(Side::LEFT,
                  shouldFlipForSide(Side::LEFT, command) ? ArmPose::FLIP : ArmPose::SCAN);
      break;
    default:
      break;
  }

  switch (command.rightIntent) {
    case ActionIntent::GRAB:
      rightTargetPulse = config::SERVO_PULSE_MIN;
      pumpMask_ |= 0x0010;
      break;
    case ActionIntent::STORE:
      rightTargetPulse = config::SERVO_PULSE_MAX;
      pumpMask_ &= static_cast<uint16_t>(~0x0010);
      break;
    case ActionIntent::FLIP:
      planArmPath(Side::RIGHT,
                  shouldFlipForSide(Side::RIGHT, command) ? ArmPose::FLIP : ArmPose::SCAN);
      break;
    default:
      break;
  }

  if (command.requestAutonomous) {
    // Keep this deterministic and easy to replace with full autonomous sequencer.
    leftTargetPulse = static_cast<uint16_t>(config::SERVO_PULSE_MIN +
                                            ((config::SERVO_PULSE_MAX - config::SERVO_PULSE_MIN) *
                                             command.autonomousSequenceId) /
                                                10U);
    rightTargetPulse = leftTargetPulse;
  }

  if (command.leftIntent == ActionIntent::GRAB) {
    planArmPath(Side::LEFT, ArmPose::SCAN);
  } else if (command.leftIntent == ActionIntent::STORE) {
    planArmPath(Side::LEFT, ArmPose::STORAGE_ENTRY);
  }

  if (command.rightIntent == ActionIntent::GRAB) {
    planArmPath(Side::RIGHT, ArmPose::SCAN);
  } else if (command.rightIntent == ActionIntent::STORE) {
    planArmPath(Side::RIGHT, ArmPose::STORAGE_END);
  }

  // Keep a direct pulse for fallback/manual override on lead arm axis.
  const auto& lch = actuator_map::ARM_SERVO[0][0][0];
  const auto& rch = actuator_map::ARM_SERVO[1][0][0];
  SelectPcaChannel(lch.pcaIndex, lch.channel, leftTargetPulse);
  SelectPcaChannel(rch.pcaIndex, rch.channel, rightTargetPulse);
  PumpWriteMask(pumpMask_);
  applyAuxOutputs(command);
}

ColorSensorsSnapshot ArmSystem::pollColors() {
  ColorSensorsSnapshot snap;
  if (!initialized_) {
    return snap;
  }

  for (uint8_t i = 0; i < ColorSensorsSnapshot::COUNT; ++i) {
    SelectColorMuxChannel(i);
    snap.samples[i] = ReadColorSample();
  }
  lastColors_ = snap;
  return snap;
}

SideActuatorState ArmSystem::leftState() const {
  return leftState_;
}

SideActuatorState ArmSystem::rightState() const {
  return rightState_;
}

}  // namespace robot::arm_system
