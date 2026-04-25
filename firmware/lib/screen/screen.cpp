#include <screen.h>
#include <TFT_eSPI.h>
#include <res/boot_image.h>
namespace {
    TFT_eSPI tft;
}

namespace robot::screen {
    bool begin() {
        pinMode(7, OUTPUT);
        digitalWrite(7, LOW);

        tft.begin();
        tft.setRotation(3);
        tft.fillScreen(TFT_BLACK);

        tft.setSwapBytes(true);
        tft.invertDisplay(true);

        tft.pushImage(0, 0, 320, 237, boot_image);

        pinMode(7, OUTPUT);
        digitalWrite(7, HIGH);

        return true;
    }
}