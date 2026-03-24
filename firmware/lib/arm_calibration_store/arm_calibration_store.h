#pragma once

#include <cstdint>

namespace robot::arm_calibration {

enum class LogicalPose : uint8_t {
  SCAN = 0,
  IDLE = 1,
  FLIP = 2,
  STORAGE_END = 3,
  STORAGE_ENTRY = 4,
};

struct ServoPoseConfig {
  uint16_t axis0 = 307;
  uint16_t axis1 = 307;
  uint16_t axis2 = 307;
};

struct ArmCalibrationProfile {
  ServoPoseConfig pose[5];
};

class CalibrationStore {
 public:
  bool begin();
  bool load(uint8_t side, uint8_t arm, ArmCalibrationProfile* outProfile);
  bool save(uint8_t side, uint8_t arm, const ArmCalibrationProfile& profile);

 private:
  bool initialized_ = false;
};

ArmCalibrationProfile DefaultProfile();

}  // namespace robot::arm_calibration
