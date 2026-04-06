#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <remote.h>
#include <ioexpander.h>
#include <pwm-controller.h>
#include <buses.h>
#include <movers.h>
#include <i2cexpander.h>
#include <queues.h>
#include <tasks.h>
#include <state.h>
#include <battery.h>
#include <color_sensors.h>
#include <Logger.h>

namespace robot {}

void setup() {
    loggerSetup();
    delay(1000); // Allow time for Serial to initialize

    info("main", "Setup started");
    
    robot::state::begin();
    robot::queues::begin();
    robot::buses::begin();
    robot::ioexpander::begin();
    robot::movers::begin();
    robot::pwmcontroller::begin();
    robot::remote::connect();
    robot::i2cexpander::begin();
    robot::battery::begin();
    robot::color_sensors::begin();

    robot::tasks::begin();
    
    IOExpanderCommand cmd{};
    cmd.pin = 7;
    cmd.level = false;
    xQueueSend(robot::queues::io_command_queue, &cmd, 0);

    CommandBatch<PWMCommand> batch{};

    PWMCommand pwmCmd{};
    pwmCmd.controller = robot::config::front_left_turner.controller;
    pwmCmd.pin = robot::config::front_left_turner.pin;
    pwmCmd.value = 20;
    
    batch.add(pwmCmd);
    
    xQueueSend(robot::queues::pwm_command_queue, &batch, 0);

}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
