#include <battery.h>
#include <config.h>
#include <buses.h>
#include <logging.h>

struct BatteryStatus {
    uint8_t percentage;
    uint16_t voltage_mv;
    
    uint16_t cell_1_voltage_mv;
    uint16_t cell_2_voltage_mv;
    uint16_t cell_3_voltage_mv;
};

namespace robot::battery {
    ADS1015 adc(robot::config::ads1015_i2c_config.address, robot::buses::get(robot::config::ads1015_i2c_config.busId));

    bool begin() {
        robot::logging::infof("battery", "init adc addr=0x%02X", robot::config::ads1015_i2c_config.address);

        if (!adc.begin()) {
            robot::logging::warn("battery", "adc begin failed");
            return false;
        }

        adc.setGain(1);
        robot::logging::info("battery", "adc ready");
        return true;
    }

    BatteryStatus getStatus() {
        BatteryStatus status;
        status.voltage_mv = adc.readADC(3) * robot::config::full_bat_voltage_ratio * 0.125; // Convert to mV
        
        status.cell_1_voltage_mv = adc.readADC(0) * robot::config::cell_1_voltage_ratio * 0.125; // Convert to mV
        status.cell_2_voltage_mv = adc.readADC(1) * robot::config::cell_2_voltage_ratio * 0.125; // Convert to mV
        status.cell_3_voltage_mv = adc.readADC(2) * robot::config::cell_3_voltage_ratio * 0.125; // Convert to mV

        status.percentage = constrain(map(status.voltage_mv, 14000, 16800, 0, 100), 0, 100);

        static uint32_t lastLogMs = 0;
        const uint32_t nowMs = millis();
        if ((nowMs - lastLogMs) >= 2000U) {
            lastLogMs = nowMs;
            robot::logging::infof("battery", "status pct=%u pack=%umV cells=%u/%u/%umV",
                                  status.percentage,
                                  status.voltage_mv,
                                  status.cell_1_voltage_mv,
                                  status.cell_2_voltage_mv,
                                  status.cell_3_voltage_mv);
        }

        return status;
    }
}