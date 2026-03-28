#include <Arduino.h>

#include <tasks.h>


void setup() {
    Serial.begin(115200);
    if (!robot::tasks::begin()) {
        Serial.println("[tasks] failed to begin");
    } else {
        Serial.println("[tasks] started successfully");
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
