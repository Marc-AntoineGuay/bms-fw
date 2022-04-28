/*
 * charger.c
 *
 *  Created on: 16 avr. 2018
 *      Author: Nicolas Juteau
 */

#include "charger.h"

inline bool _cv_average_reached_stop_charge(){
    return (g_BatteryPack.averageCv > g_Param.cv_average_stop_charge);
}

// Input voltage = 100uV units
// Output voltage expected by charger: 0.1V/byte. Eg.: Vout=3201 -> 320.1V
inline uint16_t _convert_value_to_charger_voltage(uint32_t voltage) {
    return voltage/1000;
}

// Output maximum battery pack voltage that must not be exceeded in 100uV units
inline uint32_t _calculate_maximum_allowable_battery_pack_voltage() {
    return g_Param.cv_average_stop_charge*CELL_VOLTAGES_PER_SLAVE*TOTAL_SLAVES;
}

void manage_charger_states() {
    // charger max current is calculated in manage_balancing_config
    if (charger_connected()) {
        // charger is connected
        g_masterBMS.charger_connected = true;

        // We must do absolutely nothing if the interlock circuit is not ok.
        if (check_interlock_no_error() && interlockDetect()) {
            CAN_setupBitRate_Charger(); // Setup CAN for charger (bitrate = 250k)


            if (g_BatteryPack.charging == 1) {
                // Charger connected and we are currently charging

                /* Stop charging & reset coulomb counting if the following conditions are met:
                  * - voltage returned by charger exceeds cutoff voltage
                  * - charging current is inferior to the minimum charging current??? (old bms: gCharger.mCurrent <= MinChargeCurr)
                  * - charging timeout
                  * - average cell voltage is above configured average cell voltage to reach
                 */

                if (_cv_average_reached_stop_charge()
                    // HACK FIXME
                    //|| g_Charger.voltage > MAX_CHARGER_VOLTAGE
                    //|| g_Charger.current < MIN_CHARGER_CURRENT
                    //|| g_Time > g_Param.charge_time_max // 8 hours max charging time
                    ) {
                    g_BatteryPack.charging = 0;
                    g_BatteryPack.coulomb_counter = 0; // Reset coulomb counter
                }
            } else {
                // charger newly connected or charging was stopped by software means
                // continue charging if overall bp voltage decreases under a certain threshold, that is nominal bp voltage - an offset
                if (g_BatteryPack.total_voltage < (_calculate_maximum_allowable_battery_pack_voltage() - DEFAULT_CHARGE_BP_START_OFFSET)) {
                    g_BatteryPack.charging = 1;
                }
            }
        } else {
            g_BatteryPack.charging = 0;
            // HACK Make more clean
            for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
                g_BatteryPack.slave[idx_slave].cellDischargeON = 0;
                slave_update_cell_discharge_config(idx_slave, 0);
            }
        }
    }
    else if (charger_connected() == 0 && g_BatteryPack.charging == 0) {
        // charger not connected
        // OK
        g_masterBMS.charger_connected = false;
    }
    else if (charger_connected() == 0 && g_BatteryPack.charging == 1) {
        // charger newly disconnected
        g_masterBMS.charger_connected = false;
        g_BatteryPack.charging = 0;

        // stop transmitting CAN message to charger
        // NOT NECESSARY???? The charger is unplugged...
        //msg_schedule_tx(MSG_ID_CHARGER_TRANSMIT);
        CAN_setupBitRate_Normal();

        LTC6804_remove_all_cell_discharge_config();

        // NOTE: Normal baudrate of CANbus must not be set here directly.
        // Since the CAN tx is interrupt driven, doing so will make the STOP message
        // at 500k instead of 250k. Instead, the reset to normal baudrate is done in the CAN ISR
        //CAN_setupBitRate_Normal(); // Setup CAN for normal operation (Bitrate = 500k)
    }
}

void charger_compute_maximums(struct Charger *maxs) {
    /*// TODO: validate with POC if the max cell voltage in this case shall be the max_cv or the average_cv
    uint16_t lMaxCellVolt = g_BatteryPack.max_cv;

    // Computes the percentage of the maximum current the charger can deliver in function of the current maximum cell voltage
    // Goal: the goal is to modulate the current delivered by the charger as soon as the maximum voltage of a cell approaches the maximum allowed value
    int16_t thresholdCompensationVoltage = g_Param.cv_average_stop_charge - DEFAULT_CHARGE_CV_COMPENSATION_OFFSET;

    int16_t applyVoltageCompensation = (int16_t)((int16_t)lMaxCellVolt - (int16_t)thresholdCompensationVoltage);

    int16_t compensationFactor = 100 - ((100*applyVoltageCompensation) / ((int16_t)g_Param.cv_charge_max - thresholdCompensationVoltage));

    if (compensationFactor > 100) {
        compensationFactor = 100;
    }
    else if (compensationFactor < 0) {
        compensationFactor = 0;
    }

    // XXX: Make MAX_CHARGER_CURRENT configurable per telemetry interface. MAX_CHARGER_CURRENT is in 0.1A units
    // (i.e: whether the charger is connected on 220V or 110V supply)

    // NOTE HERE WE CAN DISABLE CHARGING
    uint16_t lMaxCurrent = compensationFactor*MAX_CHARGER_CURRENT;

    // The maximum battery pack voltage to not exceed should be computed like this:
    // cv_average_stop_charge * number of cells per slave * number of slaves per segment * number of segment
    maxs->voltage = _convert_value_to_charger_voltage(_calculate_maximum_allowable_battery_pack_voltage());
    maxs->current = lMaxCurrent;
    maxs->status = !g_BatteryPack.charging; // status = 1-> stop charger*/

    if (g_BatteryPack.max_cv >= parameters.cv_charge_max && g_BatteryPack.max_cv - 2 < g_BatteryPack.min_cv)
    {
        // Fully charged
        g_BatteryPack.charging = false;
        maxs->voltage = 0;
        maxs->current = 0;
        maxs->status = 0;
    }
    else if (g_BatteryPack.max_cv >= parameters.cv_charge_max)
    {
        // Charged high enough but not balanced enough
        // This is supposed to continue balancing
        g_BatteryPack.charging = true;
        maxs->voltage = 0;
        maxs->current = 0;
        maxs->status = 0;
    }
    else
    {
        // In charging normally
        g_BatteryPack.charging = true;
        maxs->voltage = parameters.cv_charge_max * 0.1;
        maxs->current = 0x96;
        maxs->status = 1;
    }
}



