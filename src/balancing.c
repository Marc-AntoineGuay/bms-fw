 /*
 * balancing.c
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "balancing.h"

void _debug_balancing_beautiful_pattern() {
    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        uint16_t new_config = 0;
        for (uint16_t i = 0; i < CELL_VOLTAGES_PER_SLAVE; i++) {
            bool discharge = idx_slave == 8;
            new_config |= discharge << i;
        }
        g_BatteryPack.slave[idx_slave].cellDischargeON = new_config;

    }
}

void _calculate_balancing_discharge_above_average() {
    // check threshold, voltages, and configs
    uint16_t threshold;
    threshold = g_BatteryPack.averageCv; // basic balance. Discharge all above average

    uint16_t new_config;
    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        new_config = 0;
        for (uint16_t idx_cell = 0; idx_cell < CELL_VOLTAGES_PER_SLAVE; idx_cell++) {
            bool discharge = g_BatteryPack.slave[idx_slave].cv[idx_cell] > threshold && g_BatteryPack.max_cv - 100 > g_BatteryPack.min_cv;
            new_config |= (discharge << idx_cell);
        }
        g_BatteryPack.slave[idx_slave].cellDischargeON = new_config;
    }
}

void _update_balancing_config() {
    // write cellDischarge to LTC config. Will be updated on next config write
    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        slave_update_cell_discharge_config(idx_slave, g_BatteryPack.slave[idx_slave].cellDischargeON);
    }
}

/**
 * @brief: Calculate balancing resistors and set appropriate slave configurations
 *
 * Balancing is done once the maximum cell voltage reaches the threshold.
 * During balancing, the charger's current is limited to max_cell_voltage/10ohm.
 *
 * The balancing configuration requires a configuration write to all slaves (large packets).
 * Configuration writes are possibly only done during temperature measurements.
 */
void manage_balancing_config(){
    if (true) { //HACK g_BatteryPack.charging && interlockDetect()){
        // HACK Overwrite here
        g_Param.cv_balance_min_threshold = 34000;
        bool balance_cv_threshold = g_BatteryPack.max_cv >= g_Param.cv_balance_min_threshold;
        if (balance_cv_threshold)
        {
            /* _debug_balancing_beautiful_pattern(); */
            _calculate_balancing_discharge_above_average();
            _update_balancing_config();
        }
    }
}
