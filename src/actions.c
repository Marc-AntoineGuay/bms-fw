/*
 * actions.c
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "actions.h"
#include "memory.h"
#include "util.h"
#include "error_management.h"
#include "balancing.h"
#include "charger.h"


  ////////////////
 /// Helpers ///
//////////////

// XXX Multiple constants here have no explication (seems to be adc value but IDFK)
static inline void manageRawAdcValues() {
    if (g_RawAdcValues.process_groupA_ready) {
        // Current probe
        int16_t current_probe = (int16_t)g_RawAdcValues.current_probe;
        int16_t current_probe_ref = (int16_t)g_RawAdcValues.current_probe_ref;
// Remember to clear interrupts, even during premature returns.

        // check if value is out of range
        if (current_probe > 3800) {
        }
        else g_EventCount.current_probe_not_out_of_range++;

        // check if current too big ( abs(current) > 400A)
        if ((current_probe > 3517) || (current_probe < 207)) {
        }
        else g_EventCount.current_probe_not_overcurrent++;

        // check if current probe reference is in bounds
        // Put back to middle of range. Current probe 0A is at 1.5V (Max value is 3V)
        current_probe = current_probe - 1840;
        g_BatteryPack.total_current = current_probe;
        // HERE USE OFFSET
        g_BatteryPack.coulomb_counter += (int64_t) current_probe;

        // check if current probe reference is within expected bounds
        if ((current_probe_ref > 1000) && (current_probe_ref < 1700)) {
            g_EventCount.current_probe_ref_ok++;
        }

        // IMD data out
        g_masterBMS.imd_data_out = g_RawAdcValues.imd_data_out;

        // HV measurement
        g_masterBMS.hv_measurement = g_RawAdcValues.hv_measurement;

        // Master temperature
        g_masterBMS.temperature = g_RawAdcValues.master_temperature;

        g_RawAdcValues.process_groupA_ready = false;
    }

    if (g_RawAdcValues.process_groupB_ready) {

        // Power levels
        g_masterBMS.pwr_1V2 = g_RawAdcValues.pwr_1v2;
        g_masterBMS.pwr_3V3 = g_RawAdcValues.pwr_3v3;
        g_masterBMS.pwr_5V = g_RawAdcValues.pwr_5v;

        g_RawAdcValues.process_groupB_ready = false;
    }
}

static inline bool cvOutOfRange(uint16_t cv){
    bool too_big, too_small;
    too_big = cv > g_Param.cv_range_max;
    too_small = cv < g_Param.cv_range_min;
    return (too_big || too_small);
}

static inline bool cvUnderOrOverVoltage(uint16_t cv){
    bool too_big, too_small;
    if (g_BatteryPack.charging==1) {
        too_big = cv > g_Param.cv_charge_max;
        too_small = cv < g_Param.cv_charge_min;
    }
    else {
        too_big = cv > g_Param.cv_discharge_max;
        too_small = cv < g_Param.cv_discharge_min;
    }

    return (too_big || too_small);
}

static inline void check_cv_value(uint16_t idx_slave, uint16_t idx_cell, uint16_t cv_value){
    if (cvOutOfRange(cv_value)){
        // implausible cv measurement
        // Do nothing because this might not be problematic yet
    }
    else {
        g_EventCount.slave[idx_slave].cv_not_out_of_range[idx_cell]++;
    }

    if (cvUnderOrOverVoltage(cv_value)){
        // measurements plausible but not tolerated. Out of cell's tolerated cv
        // Do nothing because this might not be problematic yet
    }
    else {
        g_EventCount.slave[idx_slave].cv_not_UVOV[idx_cell]++;
    }
}

static inline bool thermistorOutOfRange(uint16_t thermistor_value){
    bool too_big, too_small;
    too_small = thermistor_value < g_Param.thermistor_range_min; // higher temperature
    too_big = thermistor_value > g_Param.thermistor_range_max; //lower temperature
    return (too_big || too_small);
}

static inline bool thermistorMaxMinOutOfRange(){
    return thermistorOutOfRange(g_BatteryPack.min_thermistor) || thermistorOutOfRange(g_BatteryPack.max_thermistor);
}

static inline bool thermistorFanStart(){
    return g_BatteryPack.max_thermistor < g_Param.thermistor_fan_start;
}

static inline bool thermistorFanStop(){
    return g_BatteryPack.max_thermistor > g_Param.thermistor_fan_stop;
}

static inline bool thermistorOverheat(uint16_t thermistor_value){
    bool overheat;

    if (g_BatteryPack.charging==1){
        overheat = thermistor_value < g_Param.thermistor_min_charging;
    }
    else {
        overheat = thermistor_value < g_Param.thermistor_min;
    }
    return overheat;
}

static inline void checkThermistorValue(uint16_t idx_slave, uint16_t idx_thermistor_id){
    uint16_t thermistor_value;

    thermistor_value = g_BatteryPack.slave[idx_slave].temperature[idx_thermistor_id];

    // all checks are counterintuitive because thermistor voltage gets smaller with higher temperature.
    if (thermistorOutOfRange(thermistor_value)){
        // wait until result value is permanent
    }
    else {
        g_EventCount.slave[idx_slave].temperature_not_out_of_range[idx_thermistor_id]++;
    }

    if (thermistorOverheat(thermistor_value)){
    }
    else {
        g_EventCount.slave[idx_slave].temperature_value_ok[idx_thermistor_id]++;
    }
}

static inline void checkThermistorValues(){
    uint16_t last_thermistor_id = g_temperatureSensorIdx;
    if (g_temperatureSensorIdx == 0) last_thermistor_id = 15;
    else last_thermistor_id = g_temperatureSensorIdx -1;

    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        checkThermistorValue(idx_slave, last_thermistor_id);
    }
}

/*
 * Update BatteryPack cell voltage related information
 *
 * Increments EventCount values with check_cv_values
 *
 * Updates average cell voltages for slaves and complete battery pack
 * Updates max cell voltage
 * Updates min cell voltage
 */
static inline void updateBatteryPackCvInfo(){
    uint32_t total_slave = 0;
    uint32_t total_pack = 0;
    uint16_t max_cv = 0;
    uint16_t min_cv = 0xFFFF;

    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        total_slave = 0; // reset slave accumulation for average
        for (uint16_t idx_cell=0; idx_cell<CELL_VOLTAGES_PER_SLAVE; idx_cell++) {
            uint16_t cell_voltage = g_BatteryPack.slave[idx_slave].cv[idx_cell];

            check_cv_value(idx_slave, idx_cell, cell_voltage);

            // average_cv: accumulate slave total voltage
            total_slave += cell_voltage;
            // max_cv: find max cell voltage
            if (max_cv < cell_voltage) max_cv = cell_voltage;
            // min_cv: find min cell voltage
            if (min_cv > cell_voltage) min_cv = cell_voltage;

        }
        // slave average_cv
        total_pack += total_slave;
        g_BatteryPack.slave[idx_slave].averageCv = total_slave / CELL_VOLTAGES_PER_SLAVE;
    }

    // total pack voltage
    g_BatteryPack.total_voltage = total_pack;
    // total pack average_cv
    g_BatteryPack.averageCv = total_pack / (TOTAL_SLAVES*CELL_VOLTAGES_PER_SLAVE);
    // write max_cv and min_cv to global value
    g_BatteryPack.max_cv = max_cv;
    g_BatteryPack.min_cv = min_cv;
}

/*
 * @brief: Update BatteryPack thermistor max and min values
 *
 * Updates max thermistor value
 * Updates min thermistor value
 */
static inline void updateBatteryPackThermistorMaxMin(){
    uint16_t max_th = 0;
    uint16_t min_th = 0xFFFF;

    for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
        for (uint16_t idx_th=0; idx_th<TEMPERATURE_SENSORS_PER_SLAVE; idx_th++) {
            uint16_t thermistor_value = g_BatteryPack.slave[idx_slave].temperature[idx_th];
            if (max_th < thermistor_value) max_th = thermistor_value;
            if (min_th > thermistor_value) min_th = thermistor_value;
        }
    }

    g_BatteryPack.max_thermistor = max_th;
    g_BatteryPack.min_thermistor = min_th;
}

static inline void manageFanStates() {
    // Hack enable fans always
    if (true || batteryPack.charging || thermistorFanStart()==1 || thermistorMaxMinOutOfRange()==1){
        G_START_FAN();
    }
    else if (thermistorFanStop()==1 && isFanRunningSoftware()==1){
        G_STOP_FAN();
    }
    manageFan();
}

/*
 * Any LED state change should be done in here or in timing interrupts
 */
static inline void manageDebugLEDs(){

    // LED 0 toggles every second in interrupt

    // LED 1
    if(g_BatteryPack.charging == 1){
        setLed1();
    }
    else{
        clearLed1();
    }

    // LED 2
    // old master BMS used to toggle displaying specific error cause
}

/**
 * Updates current probe offset value when interlock is closed
 *
 * The Hall Effect Current Sensor has an offset when 0 current.
 * There is 0 current when the interlock is open (interlock not detected).
 */
static inline void manageCurrentProbeOffset(){
    if (interlockDetect()==0){
        g_current_probe_offset = g_BatteryPack.total_current;
    }
}

  /////////////////////
 /// Implementaton ///
/////////////////////

void manageStatus(){
    manage_charger_states();
    manageRawAdcValues();
    manageDebugLEDs();
}


void perform_actions(){

    checkInterlockCircuit();

    manageCurrentProbeOffset(); // could be done less often
    check_fan();

    if (ready_read_CV_and_temperature()) {
       slave_read_config();

       slave_read_CV_and_temperature();
       updateBatteryPackCvInfo(); // updates and checks if any errors with values
       checkThermistorValues(); // checks if any errors with values

       // In ISR Now
       // manage_balancing_config();
       slave_write_current_config();

       // Cell temperatures from thermistors
       if (g_temperatureSensorIdx == 0) { // (16x new temperatures received)
           updateBatteryPackThermistorMaxMin();
       }
       manageFanStates();
    }
#ifndef USING_LAUNCHPAD
    else if (ready_save_data_eeprom()) {
       saveCheckpointToEeprom();
    }
    // XXX
#endif
}
