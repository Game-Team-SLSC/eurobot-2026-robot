#pragma once

#include "types.h"

namespace robot::radio {

class RadioReceiver {
 public:
  RadioReceiver();

  bool begin();
  bool read(RemoteData* outFrame);
  void setAck(const CommandAck& ack);
  bool isReady() const;

 private:
  class Impl;
  Impl* impl_;
};

}  // namespace robot::radio
