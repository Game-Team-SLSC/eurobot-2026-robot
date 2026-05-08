#include <screen.h>
#include <TFT_eSPI.h>
#include "res/res.h"
#include <cstring>
#include <state.h>
#include <Logger.h>
#include <spi_mutex.h>
#include <queues.h>

struct Rect {
    int16_t x1;
    int16_t y1;
    int16_t x2;
    int16_t y2;
};
namespace {
    TFT_eSPI tft;
    
    constexpr int screen_width = 320;
    constexpr int screen_height = 240;
    
    // dahsboard
    TFT_eSprite statusBg(&tft);
    TFT_eSprite statusText(&tft);

    TFT_eSprite controlBg(&tft);
    TFT_eSprite controlText(&tft);

    TFT_eSprite teamColorBg(&tft);
    TFT_eSprite teamColorImage(&tft);

    TFT_eSprite remoteFreqBg(&tft);
    TFT_eSprite remoteFreqText(&tft);

    TFT_eSprite battPrcBg(&tft);
    TFT_eSprite battPrcText(&tft);
    
    TFT_eSprite battFillBg(&tft);
    TFT_eSprite battFillImage(&tft);

    TFT_eSprite connectedIconBg(&tft);
    TFT_eSprite connectedIconImage(&tft);
    
    constexpr Rect status_rect = {64, 90, 246, 130};
    constexpr Rect control_rect = {23, 193, 119, 217};
    constexpr Rect team_color_rect = {239, 177, 293, 231};
    constexpr Rect remote_freq_rect = {148, 189, 213, 215};
    constexpr Rect batt_prc_rect = {235, 35, 275, 63};
    constexpr Rect batt_fill_rect = {287, 42, 309, 51};
    constexpr Rect connected_icon_rect = {216, 40, 235, 59};

    // battery menu

    TFT_eSprite batteryStatusBg(&tft);
    TFT_eSprite batteryStatusText(&tft);

    TFT_eSprite batteryCell0Bg(&tft);
    TFT_eSprite batteryCell1Bg(&tft);
    TFT_eSprite batteryCell2Bg(&tft);
    TFT_eSprite batteryCell3Bg(&tft);

    TFT_eSprite batteryCellFill(&tft);

    constexpr Rect battery_status_rect = {80, 40, 250, 65};
    constexpr Rect battery_cell_1_rect = {5, 105, 79, 227};
    constexpr Rect battery_cell_2_rect = {83, 105, 156, 227};
    constexpr Rect battery_cell_3_rect = {161, 105, 234, 227};
    constexpr Rect battery_cell_4_rect = {237, 105, 310, 227};

    // logs menu

    constexpr Rect logs_rect = {5, 41, 313, 238};

    uint32_t rgbToHex(byte r, byte g, byte b) {
        return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
}

namespace robot::screen {
    bool begin() {
        IOExpanderCommand cmdb{};
        cmdb.pin = 7;
        cmdb.level = true;
        xQueueSend(robot::queues::io_command_queue, &cmdb, 0);    

        delay(100); // Allow time for IO expander to process command and power up the screen

        IOExpanderCommand cmdz{};
        cmdz.pin = 7;
        cmdz.level = false;
        xQueueSend(robot::queues::io_command_queue, &cmdz, 0);            

        delay(10);

        robot::spi_mutex::Guard spiGuard;
        if (!spiGuard.isLocked()) {
            error("screen", "SPI mutex unavailable");
            return false;
        }

        tft.begin();
        tft.setRotation(3);
        tft.fillScreen(TFT_BLACK);

        tft.setSwapBytes(true);
        tft.invertDisplay(true);

        tft.pushImage(0, 0, screen_width, screen_height, robot::screen::res::boot_image, true);
        
        statusBg.createSprite(status_rect.x2 - status_rect.x1, status_rect.y2 - status_rect.y1);
        statusBg.setSwapBytes(true);
        statusBg.pushImage(-status_rect.x1, -status_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);

        statusText.createSprite(status_rect.x2 - status_rect.x1, status_rect.y2 - status_rect.y1);

        controlBg.createSprite(control_rect.x2 - control_rect.x1, control_rect.y2 - control_rect.y1);
        controlBg.setSwapBytes(true);
        controlBg.pushImage(-control_rect.x1, -control_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);

        controlText.createSprite(control_rect.x2 - control_rect.x1, control_rect.y2 - control_rect.y1);

        teamColorBg.createSprite(team_color_rect.x2 - team_color_rect.x1, team_color_rect.y2 - team_color_rect.y1);
        teamColorBg.setSwapBytes(true);
        teamColorBg.pushImage(-team_color_rect.x1, -team_color_rect.y1, 59, 59, robot::screen::res::dashboard);
        
        teamColorImage.createSprite(team_color_rect.x2 - team_color_rect.x1, team_color_rect.y2 - team_color_rect.y1);
        teamColorImage.setSwapBytes(true);

        remoteFreqBg.createSprite(remote_freq_rect.x2 - remote_freq_rect.x1, remote_freq_rect.y2 - remote_freq_rect.y1);
        remoteFreqBg.setSwapBytes(true);
        remoteFreqBg.pushImage(-remote_freq_rect.x1, -remote_freq_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);

        remoteFreqText.createSprite(remote_freq_rect.x2 - remote_freq_rect.x1, remote_freq_rect.y2 - remote_freq_rect.y1);

        battPrcBg.createSprite(batt_prc_rect.x2 - batt_prc_rect.x1, batt_prc_rect.y2 - batt_prc_rect.y1);
        battPrcBg.setSwapBytes(true);
        battPrcBg.pushImage(-batt_prc_rect.x1, -batt_prc_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);
        
        battPrcText.createSprite(batt_prc_rect.x2 - batt_prc_rect.x1, batt_prc_rect.y2 - batt_prc_rect.y1);

        battFillBg.createSprite(batt_fill_rect.x2 - batt_fill_rect.x1, batt_fill_rect.y2 - batt_fill_rect.y1);
        battFillBg.setSwapBytes(true);
        battFillBg.pushImage(-batt_fill_rect.x1, -batt_fill_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);

        battFillImage.createSprite(batt_fill_rect.x2 - batt_fill_rect.x1, batt_fill_rect.y2 - batt_fill_rect.y1);
        battFillImage.setSwapBytes(true);

        connectedIconBg.createSprite(connected_icon_rect.x2 - connected_icon_rect.x1, connected_icon_rect.y2 - connected_icon_rect.y1);
        connectedIconBg.setSwapBytes(true);
        connectedIconBg.pushImage(-connected_icon_rect.x1, -connected_icon_rect.y1, screen_width, screen_height, robot::screen::res::dashboard);

        connectedIconImage.createSprite(connected_icon_rect.x2 - connected_icon_rect.x1, connected_icon_rect.y2 - connected_icon_rect.y1);
        connectedIconImage.setSwapBytes(true);

        // battery menu

        batteryStatusBg.createSprite(battery_status_rect.x2 - battery_status_rect.x1, battery_status_rect.y2 - battery_status_rect.y1);
        batteryStatusBg.setSwapBytes(true);
        batteryStatusBg.pushImage(-battery_status_rect.x1, -battery_status_rect.y1, screen_width, screen_height, robot::screen::res::battery_menu);

        batteryStatusText.createSprite(battery_status_rect.x2 - battery_status_rect.x1, battery_status_rect.y2 - battery_status_rect.y1);

        batteryCell0Bg.createSprite(battery_cell_1_rect.x2 - battery_cell_1_rect.x1, battery_cell_1_rect.y2 - battery_cell_1_rect.y1);
        batteryCell0Bg.setSwapBytes(true);
        batteryCell0Bg.pushImage(-battery_cell_1_rect.x1, -battery_cell_1_rect.y1, screen_width, screen_height, robot::screen::res::battery_menu);

        batteryCell1Bg.createSprite(battery_cell_2_rect.x2 - battery_cell_2_rect.x1, battery_cell_2_rect.y2 - battery_cell_2_rect.y1);
        batteryCell1Bg.setSwapBytes(true);
        batteryCell1Bg.pushImage(-battery_cell_2_rect.x1, -battery_cell_2_rect.y1, screen_width, screen_height, robot::screen::res::battery_menu);

        batteryCell2Bg.createSprite(battery_cell_3_rect.x2 - battery_cell_3_rect.x1, battery_cell_3_rect.y2 - battery_cell_3_rect.y1);
        batteryCell2Bg.setSwapBytes(true);
        batteryCell2Bg.pushImage(-battery_cell_3_rect.x1, -battery_cell_3_rect.y1, screen_width, screen_height, robot::screen::res::battery_menu);

        batteryCell3Bg.createSprite(battery_cell_4_rect.x2 - battery_cell_4_rect.x1, battery_cell_4_rect.y2 - battery_cell_4_rect.y1);
        batteryCell3Bg.setSwapBytes(true);
        batteryCell3Bg.pushImage(-battery_cell_4_rect.x1, -battery_cell_4_rect.y1, screen_width, screen_height, robot::screen::res::battery_menu);

        batteryCellFill.createSprite(battery_cell_1_rect.x2 - battery_cell_1_rect.x1, battery_cell_1_rect.y2 - battery_cell_1_rect.y1);
        batteryCellFill.setSwapBytes(true);

        return true;
    }

    void updateStatus(const char* status, uint16_t color) {
        statusBg.pushToSprite(&statusText, 0, 0);
        statusText.setTextColor(color);
        statusText.setTextDatum(MC_DATUM);
        statusText.setFreeFont(&FreeSans12pt7b);
        statusText.drawString(status, (status_rect.x2 - status_rect.x1) / 2, (status_rect.y2 - status_rect.y1) / 2);
        
        robot::spi_mutex::Guard spiGuard;
        if (spiGuard.isLocked()) {
            statusText.pushSprite(status_rect.x1, status_rect.y1, TFT_BLACK);
        }
    }

    void updateControl(const char* control) {
        controlBg.pushToSprite(&controlText, 0, 0);
        controlText.setTextColor(TFT_GREEN);
        controlText.setTextDatum(MC_DATUM);
        controlText.setFreeFont(&FreeSans9pt7b);
        controlText.drawString(control, (control_rect.x2 - control_rect.x1) / 2, (control_rect.y2 - control_rect.y1) / 2);
        
        robot::spi_mutex::Guard spiGuard;
        if (spiGuard.isLocked()) {
            controlText.pushSprite(control_rect.x1, control_rect.y1, TFT_BLACK);
        }
    }

    void updateTeamColor(bool isYellow) {
        teamColorBg.pushToSprite(&teamColorImage, 0, 0);
        if (isYellow) {
            teamColorImage.pushImage(0, 0, 59, 59, robot::screen::res::yellow_team_icon);
        } else {
            teamColorImage.pushImage(0, 0, 59, 59, robot::screen::res::blue_team_icon);
        }
        
        robot::spi_mutex::Guard spiGuard;
        if (spiGuard.isLocked()) {
            teamColorImage.pushSprite(team_color_rect.x1, team_color_rect.y1, TFT_BLACK);
        }
    }

    void updateBatteryStatus(robot::battery::BatteryStatus &status) {
        batteryStatusBg.pushToSprite(&batteryStatusText, 0, 0);

        batteryStatusText.setTextColor(status.percentage <= 5 ? TFT_RED : TFT_WHITE);
        batteryStatusText.setTextDatum(ML_DATUM);
        batteryStatusText.setTextSize(1);
        batteryStatusText.setFreeFont(&FreeSans12pt7b);
        batteryStatusText.drawString((std::to_string(status.percentage) + "%").c_str(), 0, (battery_status_rect.y2 - battery_status_rect.y1) / 2);

        batteryStatusText.setTextDatum(BR_DATUM);
        batteryStatusText.setTextFont(1);
        batteryStatusText.setTextSize(2);

        char buf[16];

        if (status.voltage_mv <= 1000) {
            std::strncpy(buf, "UNPLUGGED", sizeof(buf));
        } else {
            std::snprintf(buf, sizeof(buf), "%.1fV", status.voltage_mv / 1000.0f);
        }

        batteryStatusText.drawString(
            buf, 
            battery_status_rect.x2 - battery_status_rect.x1,
            battery_status_rect.y2 - battery_status_rect.y1
        );

        robot::spi_mutex::Guard spiGuard;
        if (!spiGuard.isLocked()) {
            return;
        }

        batteryStatusText.pushSprite(battery_status_rect.x1, battery_status_rect.y1);

        // cells

        
        uint8_t cell1maxY = map(100 - status.cell_1_percentage, 0, 100, 0, battery_cell_1_rect.y2 - battery_cell_1_rect.y1);
        
        batteryCell0Bg.pushToSprite(&batteryCellFill, 0, 0);
        batteryCellFill.pushImage(0, cell1maxY, 76, 125 - cell1maxY, robot::screen::res::battery_cell_fill);
        
        batteryCellFill.setTextColor(TFT_WHITE);
        batteryCellFill.setTextDatum(BC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans12pt7b);
        batteryCellFill.drawString((std::to_string(status.cell_1_percentage) + "%").c_str(), (battery_cell_1_rect.x2 - battery_cell_1_rect.x1) / 2, (battery_cell_1_rect.y2 - battery_cell_1_rect.y1) / 2);

        batteryCellFill.setTextDatum(TC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans9pt7b);

        // prints cell voltage 
        std::snprintf(buf, sizeof(buf), "%.2fV", status.cell_1_voltage_mv / 1000.0f);
        batteryCellFill.drawString(buf, (battery_cell_1_rect.x2 - battery_cell_1_rect.x1) / 2, (battery_cell_1_rect.y2 - battery_cell_1_rect.y1)/2);
        
        batteryCellFill.pushSprite(battery_cell_1_rect.x1, battery_cell_1_rect.y1);

        uint8_t cell2maxY = map(100 - status.cell_2_percentage, 0, 100, 0, battery_cell_2_rect.y2 - battery_cell_2_rect.y1);
        
        batteryCell1Bg.pushToSprite(&batteryCellFill, 0, 0);
        batteryCellFill.pushImage(0, cell2maxY, 76, 125 - cell2maxY, robot::screen::res::battery_cell_fill);
        
        batteryCellFill.setTextColor(TFT_WHITE);
        batteryCellFill.setTextDatum(BC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans12pt7b);
        batteryCellFill.drawString((std::to_string(status.cell_2_percentage) + "%").c_str(), (battery_cell_2_rect.x2 - battery_cell_2_rect.x1) / 2, (battery_cell_2_rect.y2 - battery_cell_2_rect.y1) / 2);

        batteryCellFill.setTextDatum(TC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans9pt7b);

        // prints cell voltage
        std::snprintf(buf, sizeof(buf), "%.2fV", status.cell_2_voltage_mv / 1000.0f);
        batteryCellFill.drawString(buf, (battery_cell_2_rect.x2 - battery_cell_2_rect.x1) / 2, (battery_cell_2_rect.y2 - battery_cell_2_rect.y1) / 2);
        
        batteryCellFill.pushSprite(battery_cell_2_rect.x1, battery_cell_2_rect.y1);

        uint8_t cell3maxY = map(100 - status.cell_3_percentage, 0, 100, 0, battery_cell_3_rect.y2 - battery_cell_3_rect.y1);
        
        batteryCell2Bg.pushToSprite(&batteryCellFill, 0, 0);
        batteryCellFill.pushImage(0, cell3maxY, 76, 125 - cell3maxY, robot::screen::res::battery_cell_fill);
        
        batteryCellFill.setTextColor(TFT_WHITE);
        batteryCellFill.setTextDatum(BC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans12pt7b);
        batteryCellFill.drawString((std::to_string(status.cell_3_percentage) + "%").c_str(), (battery_cell_3_rect.x2 - battery_cell_3_rect.x1) / 2, (battery_cell_3_rect.y2 - battery_cell_3_rect.y1) / 2);

        batteryCellFill.setTextDatum(TC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans9pt7b);

        // prints cell voltage
        std::snprintf(buf, sizeof(buf), "%.2fV", status.cell_3_voltage_mv / 1000.0f);
        batteryCellFill.drawString(buf, (battery_cell_3_rect.x2 - battery_cell_3_rect.x1) / 2, (battery_cell_3_rect.y2 - battery_cell_3_rect.y1) / 2);
        
        batteryCellFill.pushSprite(battery_cell_3_rect.x1, battery_cell_3_rect.y1);
        
        uint8_t cell4maxY = map(100 - status.cell_4_percentage, 0, 100, 0, battery_cell_4_rect.y2 - battery_cell_4_rect.y1);

        batteryCell3Bg.pushToSprite(&batteryCellFill, 0, 0);
        batteryCellFill.pushImage(0, cell4maxY, 76, 125 - cell4maxY, robot::screen::res::battery_cell_fill);
        
        batteryCellFill.setTextColor(TFT_WHITE);
        batteryCellFill.setTextDatum(BC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans12pt7b);
        batteryCellFill.drawString((std::to_string(status.cell_4_percentage) + "%").c_str(), (battery_cell_4_rect.x2 - battery_cell_4_rect.x1) / 2, (battery_cell_4_rect.y2 - battery_cell_4_rect.y1) / 2);

        batteryCellFill.setTextDatum(TC_DATUM);
        batteryCellFill.setFreeFont(&FreeSans9pt7b);

        // prints cell voltage
        std::snprintf(buf, sizeof(buf), "%.2fV", status.cell_4_voltage_mv / 1000.0f);
        batteryCellFill.drawString(buf, (battery_cell_4_rect.x2 - battery_cell_4_rect.x1) / 2, (battery_cell_4_rect.y2 - battery_cell_4_rect.y1) / 2);
        
        batteryCellFill.pushSprite(battery_cell_4_rect.x1, battery_cell_4_rect.y1);
    }

    void updateBatteryPercentage(uint8_t percentage) {
        battPrcBg.pushToSprite(&battPrcText, 0, 0);
        battPrcText.setTextColor(percentage <= 10 ? TFT_RED : TFT_WHITE);
        battPrcText.setTextDatum(MC_DATUM);
        battPrcText.setTextSize(2);
        battPrcText.drawString((std::to_string(percentage) + "%").c_str(), (batt_prc_rect.x2 - batt_prc_rect.x1) / 2, (batt_prc_rect.y2 - batt_prc_rect.y1) / 2);
        
        uint8_t width = map(percentage, 0, 100, 0, 22);

        battFillBg.pushToSprite(&battFillImage, 0, 0);
        battFillImage.pushImage(0, 0, width, 9, robot::screen::res::batt_fill);
        
        robot::spi_mutex::Guard spiGuard;
        if (spiGuard.isLocked()) {
            battPrcText.pushSprite(batt_prc_rect.x1, batt_prc_rect.y1, TFT_BLACK);
            battFillImage.pushSprite(batt_fill_rect.x1, batt_fill_rect.y1, TFT_BLACK);
        }
    }

    void updateRemoteFreq(uint8_t freq) {
        uint8_t r,g,b;

        const uint8_t clampedFreq = min(freq, static_cast<uint8_t>(100));
        r = 255 - map(clampedFreq, 0, 100, 0, 255);
        g = map(clampedFreq, 0, 100, 0, 224);
        b = 0;
        
        remoteFreqBg.pushToSprite(&remoteFreqText, 0, 0);
        remoteFreqText.setTextColor(rgbToHex(r, g, b));
        remoteFreqText.setTextDatum(MC_DATUM);
        remoteFreqText.setFreeFont(&FreeSans9pt7b);
        remoteFreqText.drawString(freq == 0 ? "N/A" : (std::to_string(freq) + "Hz").c_str(), (remote_freq_rect.x2 - remote_freq_rect.x1) / 2, (remote_freq_rect.y2 - remote_freq_rect.y1) / 2);

        connectedIconBg.pushToSprite(&connectedIconImage, 0, 0);
        if (freq > 0) {
            connectedIconImage.pushImage(0, 0, 13, 13, robot::screen::res::connected_icon);
        } else {
            connectedIconImage.pushImage(0, 0, 13, 13, robot::screen::res::disconnected_icon);
        }

        robot::spi_mutex::Guard spiGuard;
        if (spiGuard.isLocked()) {
            remoteFreqText.pushSprite(remote_freq_rect.x1, remote_freq_rect.y1, TFT_BLACK);
            connectedIconImage.pushSprite(connected_icon_rect.x1, connected_icon_rect.y1, TFT_BLACK);
        }
    }

    bool focus(Tab tab) {
        robot::spi_mutex::Guard spiGuard;
        if (!spiGuard.isLocked()) {
            return false;
        }

        switch (tab) {
            case Tab::DASHBOARD:
                tft.pushImage(0, 0, screen_width, screen_height, robot::screen::res::dashboard, true);
                break;
            case Tab::LOGS:
                tft.pushImage(0, 0, screen_width, screen_height, robot::screen::res::logs_menu, true);
                break;
            case Tab::BATTERY:
                tft.pushImage(0, 0, screen_width, screen_height, robot::screen::res::battery_menu, true);
                break;
        }
        return true;
    }

    void updateLogs(const char logs[10][128]) {
        robot::spi_mutex::Guard spiGuard;
        if (!spiGuard.isLocked()) {
            return;
        }

        // Définit la zone de clipping à l'écran : le texte sera limité à la zone logs_rect
        tft.setViewport(logs_rect.x1, logs_rect.y1, logs_rect.x2 - logs_rect.x1, logs_rect.y2 - logs_rect.y1);
        
        // Push l'image de fond directement depuis la mémoire flash, TFT_eSPI se chargera du clipping !
        tft.pushImage(-logs_rect.x1, -logs_rect.y1, screen_width, screen_height, robot::screen::res::logs_menu);

        tft.setTextDatum(TL_DATUM);
        tft.setTextFont(1);
        tft.setTextSize(1);
        
        for (int i = 9; i >= 0; i--) {
            if (logs[i][0] == '\0') {
                break;
            }
            
            // Determine color based on first character: 0=blue, 1=yellow, 2=red
            char colorCode = logs[i][0];
            uint16_t color = TFT_WHITE; // default
            
            if (colorCode == '0') {
                color = TFT_WHITE;
            } else if (colorCode == '1') {
                color = TFT_YELLOW;
            } else if (colorCode == '2') {
                color = TFT_RED;
            }
            
            tft.setTextColor(color);
            
            // Draw from second character onwards (skip the color code)
            tft.drawString(logs[i] + 1, 5, 5 + i * 18);
        }

        tft.resetViewport();
    }
}