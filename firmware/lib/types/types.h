#pragma once

#include <Arduino.h>
#include <cstdint>

namespace robot {

enum class Side : uint8_t {
  LEFT = 0,
  RIGHT = 1,
};

enum class RobotMode : uint8_t {
  BOOT = 0,
  IDLE = 1,
  RUN = 2,
  CALIB = 3,
  EMERGENCY = 4,
};

enum class ActionIntent : uint8_t {
  NONE = 0,
  GRAB = 1,
  STORE = 2,
  FLIP = 3,
  SNOWPLOW = 4,
};

enum class StockArea : uint8_t {
  STOCK_ISLAND_ENTRY = 0,
  STOCK_ISLAND_END = 1,
  FLIPPER = 2,
};

enum class StockIslandState : uint8_t {
  LOCKED = 0,
  OPEN = 1,
};

enum class FlipperState : uint8_t {
  STRAIGHT_LOCKED = 0,
  STRAIGHT_UNLOCKED = 1,
  FOLD_LOCKED = 2,
  FOLD_UNLOCKED = 3,
};

enum class ArmPose : uint8_t {
  SCAN = 0,
  IDLE = 1,
  FLIP = 2,
  STORAGE_END = 3,
  STORAGE_ENTRY = 4,
};

enum class AckCode : uint8_t {
  OK = 0,
  REJECT_STORAGE_OCCUPIED = 1,
  REJECT_DEPENDENCY = 2,
  REJECT_BUSY = 3,
  REJECT_INVALID = 4,
};

enum class GameColor : uint8_t {
  UNKNOWN = 0,
  YELLOW = 1,
  BLUE = 2,
};

enum SWITCH_3_POS { UP, DOWN, MIDDLE };

struct JoystickData {
  byte x = 128;  // 0 to 255
  byte y = 128;  // 0 to 255
};

struct RemoteData {
  JoystickData joystickLeft;
  JoystickData joystickRight;

  bool buttons[15] = {
      false, false, false, false, false, false, false, false,
      false, false, false, false, false, false, false};
  // for each button true if pressed
  byte slider = 0;  // 0 to 255
};

struct JoystickCommand {
  uint32_t sequence = 0;
  int16_t vx = 0;
  int16_t vy = 0;
  int16_t omega = 0;
  uint8_t slider = 0;
  uint16_t buttonsMask = 0;
  bool enableLeft = false;
  bool enableRight = false;
  bool requestAutonomous = false;
  uint8_t autonomousSequenceId = 0;
  bool requestColorBasedFlip = false;
  GameColor targetColor = GameColor::YELLOW;
  bool requestStore = false;
  StockArea requestedStoreArea = StockArea::STOCK_ISLAND_ENTRY;
  ActionIntent leftIntent = ActionIntent::NONE;
  ActionIntent rightIntent = ActionIntent::NONE;
};

struct CommandAck {
  uint32_t sequence = 0;
  bool accepted = true;
  AckCode code = AckCode::OK;
};

struct SideActuatorState {
  ArmPose armPose = ArmPose::IDLE;
  bool suctionEnabled = false;
  StockIslandState stockIsland = StockIslandState::LOCKED;
  FlipperState flipper = FlipperState::STRAIGHT_LOCKED;
};

struct StorageState {
  bool stockIslandEntryOccupied = false;
  bool stockIslandEndOccupied = false;
  bool flipperOccupied = false;
};

struct BatteryStatus {
  float packVoltage = 0.0F;
  float cell1 = 0.0F;
  float cell2 = 0.0F;
  float cell3 = 0.0F;
  float cell4 = 0.0F;
  uint8_t percent = 0;
  bool warning = false;
  bool critical = false;
};

struct ColorSample {
  uint16_t clear = 0;
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  GameColor color = GameColor::UNKNOWN;
  bool valid = false;
};

struct ColorSensorsSnapshot {
  static constexpr uint8_t COUNT = 8;
  ColorSample samples[COUNT];
};

struct MotionTarget {
  int16_t vx = 0;
  int16_t vy = 0;
  int16_t omega = 0;
};

struct SystemSnapshot {
  uint32_t uptimeMs = 0;
  uint32_t lastRfRxMs = 0;
  RobotMode mode = RobotMode::BOOT;
  bool rfLinkAlive = false;
  BatteryStatus battery;
  ColorSensorsSnapshot colors;
  MotionTarget target;
  JoystickCommand lastCommand;
  CommandAck lastAck;
  SideActuatorState leftActuator;
  SideActuatorState rightActuator;
  StorageState storage;
};

}  // namespace robot
