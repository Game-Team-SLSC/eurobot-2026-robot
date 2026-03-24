#include "arm_calibration_store.h"

#include <Preferences.h>

namespace robot::arm_calibration {

namespace {

const char* kNs = "armcal";

void FillDefault(ArmCalibrationProfile* profile) {
  *profile = DefaultProfile();
}

}  // namespace

ArmCalibrationProfile DefaultProfile() {
  ArmCalibrationProfile p;
  p.pose[static_cast<uint8_t>(LogicalPose::SCAN)] = {260, 350, 307};
  p.pose[static_cast<uint8_t>(LogicalPose::IDLE)] = {307, 307, 307};
  p.pose[static_cast<uint8_t>(LogicalPose::FLIP)] = {420, 220, 380};
  p.pose[static_cast<uint8_t>(LogicalPose::STORAGE_END)] = {330, 420, 250};
  p.pose[static_cast<uint8_t>(LogicalPose::STORAGE_ENTRY)] = {250, 430, 280};
  return p;
}

bool CalibrationStore::begin() {
  initialized_ = true;
  return true;
}

bool CalibrationStore::load(uint8_t side, uint8_t arm, ArmCalibrationProfile* outProfile) {
  if (!initialized_ || outProfile == nullptr) {
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(kNs, true)) {
    FillDefault(outProfile);
    return false;
  }

  char key[8];
  snprintf(key, sizeof(key), "s%ua%u", side, arm);
  size_t read = prefs.getBytes(key, outProfile, sizeof(ArmCalibrationProfile));
  prefs.end();

  if (read != sizeof(ArmCalibrationProfile)) {
    FillDefault(outProfile);
    return false;
  }
  return true;
}

bool CalibrationStore::save(uint8_t side, uint8_t arm, const ArmCalibrationProfile& profile) {
  if (!initialized_) {
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(kNs, false)) {
    return false;
  }
  char key[8];
  snprintf(key, sizeof(key), "s%ua%u", side, arm);
  const size_t written = prefs.putBytes(key, &profile, sizeof(ArmCalibrationProfile));
  prefs.end();
  return written == sizeof(ArmCalibrationProfile);
}

}  // namespace robot::arm_calibration
