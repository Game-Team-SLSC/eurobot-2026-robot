#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "event_bus.h"
#include "hardware_pins.h"
#include "system_state.h"
#include "task_context.h"
#include "tasks.h"

namespace {

robot::EventBus g_eventBus;
robot::SystemState g_systemState;
robot::TaskContext g_taskContext;

void CreateRuntimeTasks(robot::TaskContext* context) {
	xTaskCreatePinnedToCore(robot::tasks::CommTask, "comm", 4096, context, 4, nullptr, 1);
	xTaskCreatePinnedToCore(robot::tasks::SensorTask, "sensor", 4096, context, 3, nullptr, 1);
	xTaskCreatePinnedToCore(robot::tasks::ControlTask, "control", 4096, context, 5, nullptr, 0);
	xTaskCreatePinnedToCore(robot::tasks::UiTask, "ui", 4096, context, 2, nullptr, 1);
	xTaskCreatePinnedToCore(robot::tasks::SafetyTask, "safety", 4096, context, 6, nullptr, 0);
}

}  // namespace

void setup() {
	Serial.begin(robot::config::SERIAL_BAUDRATE);
	delay(200);
	Serial.println("[boot] eurobot firmware skeleton start");
	Serial.printf("[boot] pins spi(sck=%d miso=%d mosi=%d) rf24(ce=%d csn=%d irq=%d) i2c(sensor=%d/%d act=%d/%d)\n",
	             robot::pins::spi::SCK,
	             robot::pins::spi::MISO,
	             robot::pins::spi::MOSI,
	             robot::pins::spi::RF24_CE,
	             robot::pins::spi::RF24_CSN,
	             robot::pins::spi::RF24_IRQ,
	             robot::pins::i2c::SENSORS_SDA,
	             robot::pins::i2c::SENSORS_SCL,
	             robot::pins::i2c::ACTUATORS_SDA,
	             robot::pins::i2c::ACTUATORS_SCL);

	const bool stateReady = g_systemState.begin();
	const bool busReady = g_eventBus.begin(robot::config::EVENT_QUEUE_LENGTH);
	if (!stateReady || !busReady) {
		Serial.println("[boot] fatal: core init failed");
		while (true) {
			delay(1000);
		}
	}

	g_systemState.setMode(robot::RobotMode::IDLE);

	g_taskContext.bus = &g_eventBus;
	g_taskContext.state = &g_systemState;

	CreateRuntimeTasks(&g_taskContext);
	Serial.println("[boot] tasks started");
}

void loop() {
	// Runtime is task-driven. Keep Arduino loop idle.
	vTaskDelay(pdMS_TO_TICKS(1000));
}

