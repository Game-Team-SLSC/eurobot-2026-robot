#include <tasks.h>
#include <Arduino.h>
#include <queues.h>
#include <commands.h>
#include <state.h>

namespace {
    constexpr uint8_t PUMPS_ALL_MASK = 0x0F; // 4 pompes

    void togglePumps(uint8_t state) {
        state &= PUMPS_ALL_MASK;

        IOExpanderCommand cmd2;
        cmd2.expander = robot::config::IOExpander::KINETIC;
        cmd2.pin = 7;
        cmd2.level = (state & (1 << 1)) != 0;
    
        IOExpanderCommand cmd3;
        cmd3.expander = robot::config::IOExpander::KINETIC;
        cmd3.pin = 6;
        cmd3.level = (state & (1 << 0)) != 0;

        IOExpanderCommand cmd4;
        cmd4.expander = robot::config::IOExpander::KINETIC;
        cmd4.pin = 13;
        cmd4.level = (state & (1 << 3)) != 0;
        
        IOExpanderCommand cmd5;
        cmd5.expander = robot::config::IOExpander::KINETIC;
        cmd5.pin = 12;
        cmd5.level = (state & (1 << 2)) != 0;

        // print the state and the "<<" for each
        Serial.printf("[togglePumps] Setting pumps state: 0b%d%d%d%d (cmd2=%d cmd3=%d cmd4=%d cmd5=%d)\n",
                  (state >> 3) & 1, (state >> 2) & 1, (state >> 1) & 1, state & 1,
                  cmd2.level, cmd3.level, cmd4.level, cmd5.level);
        
        xQueueSend(robot::queues::io_command_queue, &cmd2, 0);
        xQueueSend(robot::queues::io_command_queue, &cmd3, 0);
        xQueueSend(robot::queues::io_command_queue, &cmd4, 0);
        xQueueSend(robot::queues::io_command_queue, &cmd5, 0);
    }

    bool isOurTeam(ColorResponse &color) {
        if (color.s < 0.2f) return false;

        GlobalState state = robot::state::get();

        if ((color.h >= 60.0f) && (color.h <= 90.0f)) {
            return state.isYellowTeam;
        } else {
            return !state.isYellowTeam;
        }
    }
}

namespace robot::tasks {
void control_action_task(void* parameter) {
    (void) parameter;

    Serial.println("[control_action_task]: Task started");

    while (true) {
        uint8_t action;
        if (xQueueReceive(robot::queues::action_command_queue, &action, pdMS_TO_TICKS(100)) == pdPASS) {
            robot::state::setAction(static_cast<robot::config::Action>(action));
            MotionCommand stopCmd;
            stopCmd.target = {0, 0, 0};
            xQueueSend(robot::queues::motion_command_queue, &stopCmd, 0);
            switch (static_cast<robot::config::Action>(action)) {
                case robot::config::Action::TURN:
                    {
                        CommandBatch<PWMCommand> pwmBatch;

                        // put in "saisie" position

                        PWMCommand cmd;
                        cmd.controller = robot::config::front_left_turner.controller;
                        cmd.pin = robot::config::front_left_turner.pin;
                        cmd.value = 163;
                        pwmBatch.add(cmd);

                        PWMCommand cmd2;
                        cmd2.controller = robot::config::front_right_turner.controller;
                        cmd2.pin = robot::config::front_right_turner.pin;
                        cmd2.value = 17;
                        pwmBatch.add(cmd2);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
                        
                        vTaskDelay(pdMS_TO_TICKS(500));

                        ColorCommand erColorCmd;
                        erColorCmd.sensor = robot::config::ColorSensor::FER;
                        xQueueSend(robot::queues::color_command_queue, &erColorCmd, 0);

                        ColorResponse erColorResp{};
                        const bool erOk = xQueueReceive(robot::queues::color_response_queue, &erColorResp, pdMS_TO_TICKS(1500)) == pdPASS;
                        
                        ColorCommand irColorCmd;
                        irColorCmd.sensor = robot::config::ColorSensor::FIR;
                        xQueueSend(robot::queues::color_command_queue, &irColorCmd, 0);

                        ColorResponse irColorResp{};
                        const bool irOk = xQueueReceive(robot::queues::color_response_queue, &irColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

                        ColorCommand elColorCmd;
                        elColorCmd.sensor = robot::config::ColorSensor::FEL;
                        xQueueSend(robot::queues::color_command_queue, &elColorCmd, 0);

                        ColorResponse elColorResp{};
                        const bool elOk = xQueueReceive(robot::queues::color_response_queue, &elColorResp, pdMS_TO_TICKS(1500)) == pdPASS;
                        
                        ColorCommand ilColorCmd;
                        ilColorCmd.sensor = robot::config::ColorSensor::FIL;
                        xQueueSend(robot::queues::color_command_queue, &ilColorCmd, 0);

                        ColorResponse ilColorResp{};
                        const bool ilOk = xQueueReceive(robot::queues::color_response_queue, &ilColorResp, pdMS_TO_TICKS(1500)) == pdPASS;

                        // print isYellow for each with the name of the sensor :

                        Serial.printf("[control_action_task] Color sensor readings:\n");
                        Serial.printf("  FER: h=%.1f s=%.2f v=%.2f isYellow=%s\n", erColorResp.h, erColorResp.s, erColorResp.v, isOurTeam(erColorResp) ? "true" : "false");
                        Serial.printf("  FIR: h=%.1f s=%.2f v=%.2f isYellow=%s\n", irColorResp.h, irColorResp.s, irColorResp.v, isOurTeam(irColorResp) ? "true" : "false");
                        Serial.printf("  FEL: h=%.1f s=%.2f v=%.2f isYellow=%s\n", elColorResp.h, elColorResp.s, elColorResp.v, isOurTeam(elColorResp) ? "true" : "false");
                        Serial.printf("  FIL: h=%.1f s=%.2f v=%.2f isYellow=%s\n", ilColorResp.h, ilColorResp.s, ilColorResp.v, isOurTeam(ilColorResp) ? "true" : "false");

                        uint8_t pumpsMask = 0;
                        if (erOk && isOurTeam(erColorResp)) pumpsMask |= 1 << 0;
                        if (irOk && isOurTeam(irColorResp)) pumpsMask |= 1 << 1;
                        if (elOk && isOurTeam(elColorResp)) pumpsMask |= 1 << 2;
                        if (ilOk && isOurTeam(ilColorResp)) pumpsMask |= 1 << 3;

                        Serial.printf("[control_action_task] Pumps mask: 0b%c%c%c%c\n",
                                      (pumpsMask & (1 << 3)) ? '1' : '0',
                                      (pumpsMask & (1 << 2)) ? '1' : '0',
                                      (pumpsMask & (1 << 1)) ? '1' : '0',
                                      (pumpsMask & (1 << 0)) ? '1' : '0');
                        
                        togglePumps(pumpsMask);

                        vTaskDelay(pdMS_TO_TICKS(200));

                        pwmBatch.clear();
                        

                        // Retract arm

                        PWMCommand cmd1;
                        cmd1.controller = robot::config::front_left_turner.controller;
                        cmd1.pin = robot::config::front_left_turner.pin;
                        cmd1.value = 20;
                        pwmBatch.add(cmd1);

                        PWMCommand cmd3;
                        cmd3.controller = robot::config::front_right_turner.controller;
                        cmd3.pin = robot::config::front_right_turner.pin;
                        cmd3.value = 160;
                        pwmBatch.add(cmd3);
                        
                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

                        vTaskDelay(pdMS_TO_TICKS(300));

                        // pump off

                        togglePumps(false);

                        vTaskDelay(pdMS_TO_TICKS(1000));

                        MotionCommand motionCmd;
                        motionCmd.target = {-70, 0, 0};
                        xQueueSend(robot::queues::motion_command_queue, &motionCmd, 0);

                        uint8_t idleAction = static_cast<uint8_t>(robot::config::Action::IDLE);
                        xQueueSend(robot::queues::action_command_queue, &idleAction, 0);

                        break;
                 }
                case robot::config::Action::STOCK:
                    {
                        CommandBatch<PWMCommand> pwmBatch;

                        // put in "saisie" position

                        PWMCommand cmd;
                        cmd.controller = robot::config::front_left_turner.controller;
                        cmd.pin = robot::config::front_left_turner.pin;
                        cmd.value = 180;
                        pwmBatch.add(cmd);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

                        //  pump on (toutes les pompes)
                        togglePumps(PUMPS_ALL_MASK);
                        
                        vTaskDelay(pdMS_TO_TICKS(700));
                        
                        vTaskDelay(pdMS_TO_TICKS(700));
                        
                        pwmBatch.clear();
                        
                        // Retract arm

                        PWMCommand cmd1;
                        cmd1.controller = robot::config::front_left_turner.controller;
                        cmd1.pin = robot::config::front_left_turner.pin;
                        cmd1.value = 20;
                        pwmBatch.add(cmd1);

                        PWMCommand cmd3;
                        cmd3.controller = robot::config::front_right_turner.controller;
                        cmd3.pin = robot::config::front_right_turner.pin;
                        cmd3.value = 160;
                        pwmBatch.add(cmd3);

                        
                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

                        vTaskDelay(pdMS_TO_TICKS(100));

                        uint8_t idleAction = static_cast<uint8_t>(robot::config::Action::IDLE);
                        xQueueSend(robot::queues::action_command_queue, &idleAction, 0);

                        break;
                    }
                case robot::config::Action::RELEASE:
                    {
                        Serial.println("[control_action_task] RELEASE action received");
                        CommandBatch<PWMCommand> pwmBatch;

                        // put in "saisie" position

                        PWMCommand cmd;
                        cmd.controller = robot::config::front_left_turner.controller;
                        cmd.pin = robot::config::front_left_turner.pin;
                        cmd.value = 180;
                        pwmBatch.add(cmd);

                        PWMCommand cmd2;
                        cmd2.controller = robot::config::front_right_turner.controller;
                        cmd2.pin = robot::config::front_right_turner.pin;
                        cmd2.value = 20;
                        pwmBatch.add(cmd2);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);

                        vTaskDelay(pdMS_TO_TICKS(700));

                        // pump off

                        togglePumps(false);
                        
                        vTaskDelay(pdMS_TO_TICKS(700));

                        // Retract arm

                        pwmBatch.clear();

                        PWMCommand cmd1;
                        cmd1.controller = robot::config::front_left_turner.controller;
                        cmd1.pin = robot::config::front_left_turner.pin;
                        cmd1.value = 20;
                        pwmBatch.add(cmd1);

                        PWMCommand cmd3;
                        cmd3.controller = robot::config::front_right_turner.controller;
                        cmd3.pin = robot::config::front_right_turner.pin;
                        cmd3.value = 160;
                        pwmBatch.add(cmd3);

                        xQueueSend(robot::queues::pwm_command_queue, &pwmBatch, 0);
                        
                        uint8_t idleAction = static_cast<uint8_t>(robot::config::Action::IDLE);
                        xQueueSend(robot::queues::action_command_queue, &idleAction, 0);

                        break;
                    }
                default:
                    Serial.println("[control_action_task] Unknown action command received");
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
}