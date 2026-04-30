#include <tasks.h>
#include <Arduino.h>
#include <state.h>
#include <Logger.h>
#include <FreeRTOS.h>
#include <screen.h>
#include <queues.h>

constexpr uint16_t refresh_rate = 1000 / 30; // 30 FPS

namespace robot::tasks {
    void render_ui_task(void* parameter) {
        (void) parameter;

        info("render_ui_task", "Task started");

        GlobalState lastState = robot::state::get();

        robot::screen::focus(robot::screen::Tab::DASHBOARD);
        robot::screen::updateStatus("OPERATIONAL");
        robot::screen::updateTeamColor(lastState.isYellowTeam);
        robot::screen::updateControl(lastState.action == Action::IDLE ? "MANUAL": "AUTO");
        robot::screen::updateRemoteFreq(0);
        robot::screen::updateBatteryPercentage(100);

        char logHistory[10][128];
        uint8_t nextIndex = 0;
        uint8_t totalLogs = 0;
        bool logsUpdated = false;

        while (true) {
            char* logMsg;
            if (xQueueReceive(robot::queues::logs_queue, &logMsg, 0) == pdPASS) {
                // copy
                strncpy(logHistory[nextIndex], logMsg, sizeof(logHistory[nextIndex]) - 1);
                logHistory[nextIndex][sizeof(logHistory[nextIndex]) - 1] = '\0';

                // update
                nextIndex = (nextIndex + 1) % 10;
                if (totalLogs < 10) totalLogs++;

                free(logMsg);
                logsUpdated = true;
            }
            GlobalState state = robot::state::get();

            uint8_t currentTabIndex = ((state.encoderPosition % 3) >= 0)? (state.encoderPosition % 3) : (3 + (state.encoderPosition % 3));
            uint8_t lastTabIndex = ((lastState.encoderPosition % 3) >= 0)? (lastState.encoderPosition % 3) : (3 + (lastState.encoderPosition % 3));

            robot::screen::Tab currentTab = static_cast<robot::screen::Tab>(currentTabIndex);
            robot::screen::Tab lastTab = static_cast<robot::screen::Tab>(lastTabIndex);

            if (currentTab != lastTab) {
                robot::screen::focus(currentTab);
                switch (currentTab) {
                    case robot::screen::Tab::DASHBOARD:
                        robot::screen::updateStatus("OPERATIONAL");
                        robot::screen::updateControl(state.action == Action::IDLE ? "MANUAL": "AUTO");
                        robot::screen::updateTeamColor(state.isYellowTeam);
                        robot::screen::updateRemoteFreq(state.radioFrequency);
                        robot::screen::updateBatteryPercentage(state.batteryStatus.percentage);
                        break;
                    case robot::screen::Tab::LOGS:
                        robot::screen::updateLogs(logHistory);
                        break;
                    case robot::screen::Tab::BATTERY:
                        robot::screen::updateBatteryStatus(state.batteryStatus);
                        break;
                }
            }

            switch (currentTab) {
                case robot::screen::Tab::DASHBOARD:
                    if (state.action != lastState.action) {
                        robot::screen::updateControl(state.action == Action::IDLE ? "MANUAL": "AUTO");
                    }

                    if (state.isYellowTeam != lastState.isYellowTeam) {
                        robot::screen::updateTeamColor(state.isYellowTeam);
                    }
                    
                    if (state.radioFrequency != lastState.radioFrequency) {
                        robot::screen::updateRemoteFreq(state.radioFrequency);
                    }

                    if (state.batteryStatus.percentage != lastState.batteryStatus.percentage) {
                        robot::screen::updateBatteryPercentage(state.batteryStatus.percentage);
                    }
                    break;
                case robot::screen::Tab::LOGS:
                    if (logsUpdated) {
                        robot::screen::updateLogs(logHistory);
                        logsUpdated = false;
                    }
                    break;
                case robot::screen::Tab::BATTERY:
                    robot::screen::updateBatteryStatus(state.batteryStatus);
            }

            lastState = state;
            vTaskDelay(pdMS_TO_TICKS(refresh_rate));
        }
    }     
}