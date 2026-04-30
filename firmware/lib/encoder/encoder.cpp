#include "encoder.h"
#include <commands.h>
#include <queues.h>
#include <FreeRTOS.h>
#include <Arduino.h>
#include <Logger.h>

namespace {
}

void robot::encoder::begin() {}

robot::encoder::Position robot::encoder::read() {
    IOExpanderCommand read1Cmd;
    read1Cmd.expander = robot::config::IOExpander::LOGIC;
    read1Cmd.pin = 1;
    read1Cmd.action = IOAction::READ;

    // On envoie la commande et on attend une réponse (timeout 10ms)
    xQueueSend(robot::queues::io_command_queue, &read1Cmd, 0);
    int32_t result1 = 0;
    if (xQueueReceive(robot::queues::io_response_queue, &result1, pdMS_TO_TICKS(100)) != pdPASS) {
        result1 = 0; // Sécurité si le timeout expire
    }

    IOExpanderCommand read2Cmd;
    read2Cmd.expander = robot::config::IOExpander::LOGIC;
    read2Cmd.pin = 2;
    read2Cmd.action = IOAction::READ;

    xQueueSend(robot::queues::io_command_queue, &read2Cmd, 0);
    int32_t result2 = 0;
    if (xQueueReceive(robot::queues::io_response_queue, &result2, pdMS_TO_TICKS(100)) != pdPASS) {
        result2 = 0; 
    }

    const uint8_t a = (result1 != 0) ? 1 : 0;
    const uint8_t b = (result2 != 0) ? 1 : 0;
    return static_cast<Position>((a << 1) | b);
}