#pragma once

#include <cstdint>

namespace robot::types {
    enum class SWITCH_3_POS: uint8_t {UP, DOWN, MIDDLE};

    struct JoystickData {
        uint8_t x = 128; // 0 to 255
        uint8_t y = 128; // 0 to 255
    };
    
    struct RemoteData {
        JoystickData joystickLeft{};
        JoystickData joystickRight{};

        bool buttons[15] = {
            false, false, false, false, false, false, false, false, false, false, false, false, false, false, false
        };
        // for each button true if pressed
        uint8_t slider = 0; // 0 to 255
        uint8_t score = 0;  // 0 to 255
    };

}