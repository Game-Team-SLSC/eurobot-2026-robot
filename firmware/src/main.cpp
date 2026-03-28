#include <Arduino.h>

#include <tasks.h>

void setup() {
    Serial.begin(115200);
    delay(500);

    if (!robot::tasks::begin()) {
        Serial.println("[boot] task system init failed");
    } else {
        Serial.println("[boot] task system started");
    }
}

void loop() {
    delay(1000);
}
