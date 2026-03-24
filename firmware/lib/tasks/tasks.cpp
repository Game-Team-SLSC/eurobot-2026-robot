#include "tasks.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "arm_system.h"
#include "autonomous_manager.h"
#include "config.h"
#include "motion_drive.h"
#include "remote_mapper.h"
#include "radio.h"
#include "storage_monitor.h"

namespace {

robot::radio::RadioReceiver RADIO_RECEIVER;
robot::motion_drive::MotionDrive MOTION_DRIVE;
robot::arm_system::ArmSystem ARM_SYSTEM;
robot::storage_monitor::StorageMonitor STORAGE_MONITOR;
robot::autonomous_manager::AutonomousManager AUTONOMOUS;
robot::JoystickCommand LAST_COMMAND;

uint8_t BatteryPercentFromVoltage(float voltage) {
  if (voltage <= 12.0F) {
    return 0;
  }
  if (voltage >= 16.8F) {
    return 100;
  }
  const float ratio = (voltage - 12.0F) / (16.8F - 12.0F);
  return static_cast<uint8_t>(ratio * 100.0F);
}

}  // namespace

namespace robot::tasks {

void CommTask(void* parameter) {
  auto* ctx = static_cast<TaskContext*>(parameter);
  TickType_t wake = xTaskGetTickCount();
  uint32_t seq = 0;
  const bool ready = RADIO_RECEIVER.begin();
  if (!ready) {
    Serial.println("[comm] RF24 is not ready, running without remote frames");
  }

  while (true) {
    RemoteData frame;
    if (RADIO_RECEIVER.read(&frame)) {
      const JoystickCommand cmd = remote_mapper::ToJoystickCommand(frame, seq++);
      const uint32_t nowMs = millis();

      Event event;
      event.type = EventType::RF24_COMMAND;
      event.timestampMs = nowMs;
      event.command = cmd;

      ctx->bus->publish(event, 1);
      ctx->state->setRfLinkAlive(true);
      ctx->state->setLastRfRxMs(nowMs);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::COMM_PERIOD_MS));
  }
}

void SensorTask(void* parameter) {
  auto* ctx = static_cast<TaskContext*>(parameter);
  TickType_t wake = xTaskGetTickCount();
  ARM_SYSTEM.begin();

  while (true) {
    BatteryStatus battery;
    // Placeholder until ADS1015 is wired in.
    battery.packVoltage = 15.2F;
    battery.cell1 = 3.80F;
    battery.cell2 = 3.80F;
    battery.cell3 = 3.80F;
    battery.cell4 = 3.80F;
    battery.percent = BatteryPercentFromVoltage(battery.packVoltage);
    battery.warning = battery.packVoltage < config::BATTERY_WARN_VOLTAGE;
    battery.critical = false;

    ctx->state->setBattery(battery);
    ctx->state->setColors(ARM_SYSTEM.pollColors());

    Event event;
    event.type = EventType::BATTERY_UPDATE;
    event.timestampMs = millis();
    event.battery = battery;
    ctx->bus->publish(event, 1);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::SENSOR_PERIOD_MS));
  }
}

void ControlTask(void* parameter) {
  auto* ctx = static_cast<TaskContext*>(parameter);
  TickType_t wake = xTaskGetTickCount();
  MOTION_DRIVE.begin();

  while (true) {
    const uint32_t nowMs = millis();
    Event event;
    while (ctx->bus->wait(&event, 0)) {
      if (event.type == EventType::RF24_COMMAND) {
        CommandAck ack;
        ack.sequence = event.command.sequence;
        ack.accepted = true;
        ack.code = AckCode::OK;

        LAST_COMMAND = event.command;

        if (event.command.requestAutonomous) {
          AUTONOMOUS.start(event.command.autonomousSequenceId, nowMs);
        }

        if (event.command.requestStore) {
          if (!STORAGE_MONITOR.requestStore(event.command.requestedStoreArea)) {
            ack.accepted = false;
            ack.code = AckCode::REJECT_STORAGE_OCCUPIED;
          }
        }

        if (!ack.accepted) {
          RADIO_RECEIVER.setAck(ack);
          ctx->state->setLastAck(ack);
        } else {
          RADIO_RECEIVER.setAck(ack);
          ctx->state->setLastAck(ack);
        }
      }
    }

    JoystickCommand effective = LAST_COMMAND;
    if (AUTONOMOUS.active()) {
      effective = AUTONOMOUS.update(nowMs, LAST_COMMAND);
    }

    MotionTarget target;
    target.vx = effective.vx;
    target.vy = effective.vy;
    target.omega = effective.omega;
    MOTION_DRIVE.applyTarget(target);
    ARM_SYSTEM.applyCommand(effective);
    ARM_SYSTEM.update(nowMs);

    ctx->state->setMotionTarget(target);
    ctx->state->setLastCommand(effective);
    ctx->state->setLeftActuator(ARM_SYSTEM.leftState());
    ctx->state->setRightActuator(ARM_SYSTEM.rightState());
    ctx->state->setStorage(STORAGE_MONITOR.snapshot());

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::CONTROL_PERIOD_MS));
  }
}

void UiTask(void* parameter) {
  auto* ctx = static_cast<TaskContext*>(parameter);
  TickType_t wake = xTaskGetTickCount();

  while (true) {
    const SystemSnapshot snapshot = ctx->state->getSnapshot();
    Serial.printf("[ui] mode=%u rf=%u bat=%u%% V=%.2f\n",
                  static_cast<unsigned>(snapshot.mode),
                  snapshot.rfLinkAlive ? 1U : 0U,
                  snapshot.battery.percent,
                  snapshot.battery.packVoltage);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::UI_PERIOD_MS));
  }
}

void SafetyTask(void* parameter) {
  auto* ctx = static_cast<TaskContext*>(parameter);
  TickType_t wake = xTaskGetTickCount();

  while (true) {
    const SystemSnapshot snapshot = ctx->state->getSnapshot();
    const uint32_t nowMs = millis();

    if ((nowMs - snapshot.lastRfRxMs) > config::RF_LINK_TIMEOUT_MS) {
      ctx->state->setRfLinkAlive(false);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::SAFETY_PERIOD_MS));
  }
}

}  // namespace robot::tasks
