#pragma once

#include <cstdint>

namespace robot::actuator_map {

struct PcaChannel {
  uint8_t pcaIndex;
  uint8_t channel;
};

enum class SideIndex : uint8_t {
  LEFT = 0,
  RIGHT = 1,
};

enum class ArmAxis : uint8_t {
  A0 = 0,
  A1 = 1,
  A2 = 2,
};

// Explicit mapping: [side][arm][axis]
constexpr PcaChannel ARM_SERVO[2][4][3] = {
    // LEFT side
    {{{0, 0}, {0, 1}, {0, 2}},
     {{0, 3}, {0, 4}, {0, 5}},
     {{0, 6}, {0, 7}, {0, 8}},
     {{0, 9}, {0, 10}, {0, 11}}},
    // RIGHT side
    {{{1, 0}, {1, 1}, {1, 2}},
     {{1, 3}, {1, 4}, {1, 5}},
     {{1, 6}, {1, 7}, {1, 8}},
     {{1, 9}, {1, 10}, {1, 11}}}};

constexpr PcaChannel STOCK_ISLAND_SERVO[2] = {
    {2, 2},  // LEFT
    {2, 3},  // RIGHT
};

constexpr PcaChannel FLIP_BASE_SERVO[2] = {
    {2, 4},  // LEFT
    {2, 5},  // RIGHT
};

constexpr PcaChannel FLIP_LOCK_SERVO[2] = {
    {2, 6},  // LEFT
    {2, 7},  // RIGHT
};

constexpr PcaChannel SNOWPLOW_SERVO[2] = {
    {2, 8},  // LEFT
    {2, 9},  // RIGHT
};

constexpr PcaChannel LEDS = {2, 0};
constexpr PcaChannel FANS = {2, 1};

}  // namespace robot::actuator_map
