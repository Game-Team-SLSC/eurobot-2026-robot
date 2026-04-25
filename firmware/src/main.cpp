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
#include <screen.h>
#include <actions_helpers.h>

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
    robot::screen::begin();

    delay(1000); // Allow time for peripherals to initialize

    robot::tasks::begin();
    
    IOExpanderCommand cmd{};
    cmd.pin = 7;
    cmd.level = false;
    xQueueSend(robot::queues::io_command_queue, &cmd, 0);

    

    CommandBatch<PWMCommand> batch{};

    PWMCommand pwmCmd{};
    pwmCmd.controller = robot::config::front_left_turner.controller;
    pwmCmd.pin = robot::config::front_left_turner.pin;
    pwmCmd.value = robot::actions::detail::angleToPWMValue(17);

    PWMCommand cmd3;
    cmd3.controller = robot::config::front_right_turner.controller;
    cmd3.pin = robot::config::front_right_turner.pin;
    cmd3.value = robot::actions::detail::angleToPWMValue(163);
    batch.add(cmd3);

    PWMCommand cmdala;
    cmdala.controller = robot::config::front_right_grabber.controller;
    cmdala.pin = robot::config::front_right_grabber.pin;
    cmdala.value = robot::actions::detail::angleToPWMValue(97);

    batch.add(cmdala);

    PWMCommand cmd2a;

    cmd2a.controller = robot::config::front_left_grabber.controller;
    cmd2a.pin = robot::config::front_left_grabber.pin;
    cmd2a.value = robot::actions::detail::angleToPWMValue(75);

    batch.add(cmd2a);
    
    batch.add(pwmCmd);
    
    xQueueSend(robot::queues::pwm_command_queue, &batch, 0);

}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
