#include "remote_mapper.h"

namespace {

int16_t AxisToSigned(byte value, int16_t scale) {
  const int16_t centered = static_cast<int16_t>(value) - 128;
  constexpr int16_t DEAD_BAND = 8;
  if (centered > -DEAD_BAND && centered < DEAD_BAND) {
    return 0;
  }

  // Clamp then scale from [-128,127] to [-scale,scale].
  int16_t c = centered;
  if (c < -128) {
    c = -128;
  } else if (c > 127) {
    c = 127;
  }
  return static_cast<int16_t>((static_cast<int32_t>(c) * scale) / 127);
}

robot::ActionIntent ActionFromButtons(const robot::RemoteData& frame) {
  if (frame.buttons[2]) return robot::ActionIntent::GRAB;
  if (frame.buttons[3]) return robot::ActionIntent::STORE;
  if (frame.buttons[4]) return robot::ActionIntent::FLIP;
  if (frame.buttons[5]) return robot::ActionIntent::SNOWPLOW;
  return robot::ActionIntent::NONE;
}

}  // namespace

namespace robot::remote_mapper {

JoystickCommand ToJoystickCommand(const RemoteData& frame, uint32_t sequence) {
  JoystickCommand cmd;
  cmd.sequence = sequence;

  // Left stick drives planar speed. Right X drives yaw.
  cmd.vx = AxisToSigned(frame.joystickLeft.y, 1000);
  cmd.vy = AxisToSigned(frame.joystickLeft.x, 1000);
  cmd.omega = AxisToSigned(frame.joystickRight.x, 600);

  // Side enable gates which side is authorized to execute action sequences.
  cmd.enableLeft = frame.buttons[0];
  cmd.enableRight = frame.buttons[1];
  cmd.slider = frame.slider;
  cmd.requestAutonomous = frame.buttons[14];
  cmd.autonomousSequenceId = static_cast<uint8_t>((frame.slider * 10U) / 256U);
  cmd.requestColorBasedFlip = frame.buttons[10];
  cmd.targetColor = frame.buttons[11] ? GameColor::BLUE : GameColor::YELLOW;
  cmd.requestStore = frame.buttons[9];
  if (frame.buttons[6]) {
    cmd.requestedStoreArea = StockArea::STOCK_ISLAND_ENTRY;
  } else if (frame.buttons[7]) {
    cmd.requestedStoreArea = StockArea::STOCK_ISLAND_END;
  } else if (frame.buttons[8]) {
    cmd.requestedStoreArea = StockArea::FLIPPER;
  }

  const ActionIntent selectedAction = ActionFromButtons(frame);
  cmd.leftIntent = cmd.enableLeft ? selectedAction : ActionIntent::NONE;
  cmd.rightIntent = cmd.enableRight ? selectedAction : ActionIntent::NONE;

  // Pack all buttons for future behavior mapping and telemetry.
  uint16_t mask = 0;
  for (uint8_t i = 0; i < 15; ++i) {
    if (frame.buttons[i]) {
      mask |= static_cast<uint16_t>(1U << i);
    }
  }
  cmd.buttonsMask = mask;

  return cmd;
}

}  // namespace robot::remote_mapper
