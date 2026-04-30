#include <battery.h>
#include <config.h>
#include <buses.h>
#include <Logger.h>

ADS1015 adc(robot::config::ads1015_i2c_config.address, robot::buses::get(robot::config::ads1015_i2c_config.busId));

namespace robot::battery {

    bool begin() {

        if (!adc.begin()) {
            error("battery", "Failed to initialize ADC");
            return false;
        }

        adc.setGain(1);
        info("battery", "Initialized");
        return true;
    }

    BatteryStatus getStatus() {
        BatteryStatus status;
        
        status.cell_1_voltage_mv = adc.readADC(0) * 2.f * robot::config::cell_1_voltage_ratio; // Convert to mV
        status.cell_2_voltage_mv = adc.readADC(1) * 2.f * robot::config::cell_2_voltage_ratio - status.cell_1_voltage_mv; // Convert to mV
        status.cell_3_voltage_mv = adc.readADC(2) * 2.f * robot::config::cell_3_voltage_ratio - status.cell_2_voltage_mv - status.cell_1_voltage_mv; // Convert to mV
        status.cell_4_voltage_mv = adc.readADC(3) * 2.f * robot::config::full_bat_voltage_ratio - status.cell_3_voltage_mv - status.cell_2_voltage_mv - status.cell_1_voltage_mv; // Convert to mV

        status.cell_1_percentage = constrain(map(status.cell_1_voltage_mv, 3500, 4200, 0, 100), 0, 100);
        status.cell_2_percentage = constrain(map(status.cell_2_voltage_mv, 3500, 4200, 0, 100), 0, 100);
        status.cell_3_percentage = constrain(map(status.cell_3_voltage_mv, 3500, 4200, 0, 100), 0, 100);
        status.cell_4_percentage = constrain(map(status.cell_4_voltage_mv, 3500, 4200, 0, 100), 0, 100);

        status.voltage_mv = adc.readADC(3) * 2.f * robot::config::full_bat_voltage_ratio; // Convert to mV

        // prints general voltage

        status.percentage = constrain(map(status.voltage_mv, 14000, 16800, 0, 100), 0, 100);

        return status;
    }
}