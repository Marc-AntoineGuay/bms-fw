/*
 * power_limit.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "power_limit.h"


uint16_t _min_of_3(uint16_t c1, uint16_t c2, uint16_t c3){
    if (c1<=c2 && c1<=c3){
        return c1;
    }
    else if (c2<=c1 && c2<=c3){
        return c2;
    }
    else {
        return c3;
    }
}
uint16_t _min_of_2(uint16_t c1, uint16_t c2){
    if (c1<=c2){
        return c1;
    }
    else {
        return c2;
    }
}

uint16_t _calculate_max_current_from_SOC(void) {
    // Limit based on SOC
    uint16_t coulombs_in_pack;
    uint16_t coulombs_remaining;
    coulombs_in_pack = convertPackChargeToCoulombs(g_BatteryPack.coulomb_counter);
    coulombs_remaining = coulombs_in_pack - g_Param.coulomb_count_min;
    // Power value has to be valid for the next 16 seconds
    return (coulombs_remaining / 16);
}

uint16_t _calculate_max_current_from_voltage(void) {
    // Limit based on Voltage
    // R is 12.8mV. In units of 100uOhm is 128
    return (g_BatteryPack.min_cv - g_Param.cv_discharge_min) / 128;
}

/**
 * @brief: Returns power limit on discharge
 *
 * Returns value in kW
 * Value is valid for the next 16 seconds.
 *
 * If charging, returns 0
 */
uint16_t calculatePowerLimitKW(void) {
    uint16_t current_limit_SOC;
    uint16_t current_limit_voltage;
    uint16_t pack_voltage;
    uint16_t worst_current_limit;
    uint16_t power_limit_W, power_limit_kW;

    // check if charging
    if (g_BatteryPack.charging == 1){
        return 0;
    }

    current_limit_SOC = _calculate_max_current_from_SOC();
    current_limit_voltage = _calculate_max_current_from_voltage();
    worst_current_limit = _min_of_2(current_limit_SOC, current_limit_voltage);

    pack_voltage = (uint16_t) (g_BatteryPack.total_voltage / 10000); // in Volts
    power_limit_W = worst_current_limit * pack_voltage;
    power_limit_kW = power_limit_W / 1000;

    // Check if power limit is bigger than 80kW
    power_limit_kW = _min_of_2(power_limit_kW, 80);

    return power_limit_kW;
}
