#pragma once

#include <freertos/FreeRTOS.h>

namespace robot::spi_mutex {

void begin();
bool lock(TickType_t timeoutTicks = portMAX_DELAY);
void unlock();

class Guard {
public:
    explicit Guard(TickType_t timeoutTicks = portMAX_DELAY);
    ~Guard();

    bool isLocked() const;

private:
    bool locked;
};

} // namespace robot::spi_mutex
