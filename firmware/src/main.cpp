#include <Arduino.h>
#include <SPI.h>

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
#include <encoder.h>

namespace robot {}

void setup() {
    // FORCE global 'SPI' instance to become an HSPI object inside existing RAM.
    // This allows TFT_eSPI, TMCStepper, RF24 etc. (which all hardcode "SPI")
    // to seamlessly share the single HSPI hardware seamlessly.
    SPI.end();
    SPI.~SPIClass();
    new (&SPI) SPIClass(HSPI);

    delay(1000); // Allow time for Serial to initialize
    
    robot::logger::setup();
    robot::queues::begin();  // Initialize queues BEFORE first logger call
    
    info("main", "Setup started");
    
    robot::state::begin();
    robot::buses::begin();
    robot::ioexpander::begin();
    robot::screen::begin();
    robot::movers::begin();
    robot::pwmcontroller::begin();
    robot::remote::connect();
    robot::i2cexpander::begin();
    robot::battery::begin();
    robot::color_sensors::begin();
    robot::encoder::begin();

    robot::tasks::begin();
    robot::encoder::begin();

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
    cmdala.controller = robot::config::front_grabber_left.controller;
    cmdala.pin = robot::config::front_grabber_left.pin;
    cmdala.value = robot::actions::detail::angleToPWMValue(68);

    batch.add(cmdala);

    PWMCommand cmd2a;

    cmd2a.controller = robot::config::front_right_grabber.controller;
    cmd2a.pin = robot::config::front_right_grabber.pin;
    cmd2a.value = robot::actions::detail::angleToPWMValue(75);

    batch.add(cmd2a);
    
    batch.add(pwmCmd);
    
    xQueueSend(robot::queues::pwm_command_queue, &batch, 0);

}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));
}
