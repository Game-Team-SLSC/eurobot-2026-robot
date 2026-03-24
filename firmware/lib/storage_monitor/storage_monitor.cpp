#include "storage_monitor.h"

namespace robot::storage_monitor {

bool StorageMonitor::requestStore(StockArea area) {
  switch (area) {
    case StockArea::STOCK_ISLAND_ENTRY:
      if (state_.stockIslandEntryOccupied) return false;
      state_.stockIslandEntryOccupied = true;
      return true;
    case StockArea::STOCK_ISLAND_END:
      if (state_.stockIslandEndOccupied) return false;
      state_.stockIslandEndOccupied = true;
      return true;
    case StockArea::FLIPPER:
      if (state_.flipperOccupied) return false;
      state_.flipperOccupied = true;
      return true;
    default:
      return false;
  }
}

void StorageMonitor::release(StockArea area) {
  switch (area) {
    case StockArea::STOCK_ISLAND_ENTRY:
      state_.stockIslandEntryOccupied = false;
      break;
    case StockArea::STOCK_ISLAND_END:
      state_.stockIslandEndOccupied = false;
      break;
    case StockArea::FLIPPER:
      state_.flipperOccupied = false;
      break;
  }
}

StorageState StorageMonitor::snapshot() const {
  return state_;
}

}  // namespace robot::storage_monitor
