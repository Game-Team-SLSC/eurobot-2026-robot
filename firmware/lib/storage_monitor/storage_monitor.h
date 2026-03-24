#pragma once

#include "types.h"

namespace robot::storage_monitor {

class StorageMonitor {
 public:
  bool requestStore(StockArea area);
  void release(StockArea area);
  StorageState snapshot() const;

 private:
  StorageState state_;
};

}  // namespace robot::storage_monitor
