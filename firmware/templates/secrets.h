#pragma once

#include <cstdint>

// Put this file in /include
namespace robot::secrets {
    // RF24 network configuration
    constexpr const uint8_t* rf_address = "XXXXX";
    constexpr uint8_t rf_channel = 90;
}