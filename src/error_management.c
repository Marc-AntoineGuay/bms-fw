/*
 * error_management.c
 *
 *  Created on: Apr 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "error_management.h"
#include "globals.h"
#include "memory.h"


/**
 * Clears all errors. Used when CAN msg is received.
 */
void clear_all_errors() {
    g_error_status = (uint64_t) 0;
    unprotectEepromMemory(); // XXX Check if needed
    saveErrorStatus();
}

void _reset_event_count_current_probe(){
    g_EventCount.current_probe_total = 0;
    g_EventCount.current_probe_not_out_of_range = 0;
    g_EventCount.current_probe_not_overcurrent = 0;
    g_EventCount.current_probe_ref_ok = 0;
}

void _reset_event_count_slaves(){
    g_EventCount.slave_pec_total = 0;
    g_EventCount.slave_cv_total = 0;
    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){

        g_EventCount.slave[idx_slave].cv_A_read = 0;
        g_EventCount.slave[idx_slave].cv_B_read = 0;
        g_EventCount.slave[idx_slave].cv_C_read = 0;
        g_EventCount.slave[idx_slave].cv_D_read = 0;
        g_EventCount.slave[idx_slave].pec_error = 0;

        for(uint16_t idx_thermistor=0; idx_thermistor < TEMPERATURE_SENSORS_PER_SLAVE; idx_thermistor++){
            g_EventCount.slave[idx_slave].temperature_read[idx_thermistor] = 0;
            g_EventCount.slave[idx_slave].temperature_not_out_of_range[idx_thermistor] = 0;
            g_EventCount.slave[idx_slave].temperature_value_ok[idx_thermistor] = 0;
        }

        for(uint16_t idx_cell=0; idx_cell < CELL_VOLTAGES_PER_SLAVE; idx_cell++){
            g_EventCount.slave[idx_slave].cv_not_out_of_range[idx_cell] = 0;
            g_EventCount.slave[idx_slave].cv_not_UVOV[idx_cell] = 0;
            g_EventCount.slave[idx_slave].open_wire_error[idx_cell] = 0;
        }

    }
}

bool _manage_error_current_probe(){
    // HACK No Current Probe error check
    return;

    bool out_of_range;
    bool overcurrent;
    bool interrupt_fct_not_called;
    bool ref_not_ok;

    out_of_range = g_EventCount.current_probe_not_out_of_range== 0;
    overcurrent = false; g_EventCount.current_probe_not_overcurrent==0;
    interrupt_fct_not_called = g_EventCount.current_probe_total == 0;
    ref_not_ok = g_EventCount.current_probe_ref_ok == 0;

    _reset_event_count_current_probe();

    if (interrupt_fct_not_called) set_error_bit(ERROR_CURRENT_ADC_INTERRUPT_FCT_NOT_CALLED);
    else if (out_of_range) set_error_bit(ERROR_CURRENT_PROBE_OUT_OF_RANGE);
    else if (overcurrent) set_error_bit(ERROR_CURRENT_PROBE_OVERCURRENT);
    else if (ref_not_ok) set_error_bit(ERROR_CURRENT_PROBE_REF_NOT_OK);

    return out_of_range || overcurrent || interrupt_fct_not_called || ref_not_ok;
}



bool _manage_error_slave_measurements(){

    bool has_cv_read_timeout = false;
    bool has_temp_read_timeout = false;

    uint16_t error_count = 0;

    if (g_EventCount.slave_pec_total == 0) error_count++;
    if (g_EventCount.slave_cv_total == 0) error_count++;


    // Timeout when reading Cell Voltage 
    bool not_read;
    // check if all cv_read
    bool cv_A, cv_B, cv_C, cv_D;
    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){

        cv_A = g_EventCount.slave[idx_slave].cv_A_read == 0;
        cv_B = g_EventCount.slave[idx_slave].cv_B_read == 0;
        cv_C = g_EventCount.slave[idx_slave].cv_C_read == 0;
        cv_D = g_EventCount.slave[idx_slave].cv_D_read == 0;
        not_read = cv_A || cv_B || cv_C || cv_D;

        if (not_read){
            has_cv_read_timeout = true;
            set_error_bit(ERROR_SLAVE_CV_READ_TIMEOUT);
            error_count++;
        }

    }

    bool out_of_range, bad_value, open_wire;
    bool oor_error = false;
    bool ovuv_error = false;
    // check cv UVOV and out of range
    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){
        g_BatteryPack.slave[idx_slave].voltageFlags = 0;
        for(uint16_t idx_cell=0; idx_cell < CELL_VOLTAGES_PER_SLAVE; idx_cell++){
            out_of_range = g_EventCount.slave[idx_slave].cv_not_out_of_range[idx_cell] == 0;
            bad_value = g_EventCount.slave[idx_slave].cv_not_UVOV[idx_cell] == 0;

            if (out_of_range && g_Time > 5 && g_BatteryPack.slave[idx_slave].cellDischargeON == 0){
              if (g_BatteryPack.slave[idx_slave].balancingJustFinish < 3) {
                g_BatteryPack.slave[idx_slave].balancingJustFinish++;
              } else {
                g_BatteryPack.slave[idx_slave].voltageFlags |= (uint16_t)(1 << idx_cell);
                set_error_bit(ERROR_CV_OUT_OF_RANGE);
                oor_error = true;
                error_count++;
              }
              }
              else if (bad_value && g_Time > 5 &&
                       g_BatteryPack.slave[idx_slave].cellDischargeON == 0) {
                if (g_BatteryPack.slave[idx_slave].balancingJustFinish < 3) {
                  g_BatteryPack.slave[idx_slave].balancingJustFinish++;
                } else {
                  g_BatteryPack.slave[idx_slave].voltageFlags |=
                      (uint16_t)(1 << idx_cell);
                  set_error_bit(ERROR_CV_UVOV);
                  ovuv_error = true;
                  error_count++;
                }
              }
        }
    }
    if(!oor_error) errorCVTimer.oor = 0;
    if(!ovuv_error) errorCVTimer.ovuv = 0;

#define NDIS 6
    static uint16_t numdis = 0;
    static uint16_t slavedis[NDIS];
    static uint16_t thermistordis[NDIS];
    // check temperature not out of range and value ok


    // uint16_t skipErrors = 0;
    // double average = 12365;
#define ERROR_LEN_FIX 6

#define MAX_FILTER 6

    // HACK CHECK WHICH ONES TO SKIP
    /* for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){ */
    /*         g_BatteryPack.slave[idx_slave].temperatureFlags = 0; */
    /*         for(uint16_t idx_thermistor=0; idx_thermistor < TEMPERATURE_SENSORS_PER_SLAVE; idx_thermistor++){ */
    /*             out_of_range = g_EventCount.slave[idx_slave].temperature_not_out_of_range[idx_thermistor] == 0; */
    /*             bad_value = g_EventCount.slave[idx_slave].temperature_value_ok[idx_thermistor] ==  0; */
    /*             not_read = g_EventCount.slave[idx_slave].temperature_read[idx_thermistor] == 0; */
    /*             if (out_of_range || bad_value || not_read) { */
    /*                    g_BatteryPack.slave[idx_slave].temperature[idx_thermistor] = average; */
    /*             } */
    /*         } */
    /*         // check if average is too big */
    /*         // set_error_bit(ERROR_TEMP_OVER_60C); */
    /*         // clear error bit if temp has gone down */
    /* } */

    #if HACK_USE_FILTER_TEMP
    static bool tempDisable [ TOTAL_SLAVES * TEMPERATURE_SENSORS_PER_SLAVE ] =
      #include "disabletemp.h"
      ;
    #endif


    uint16_t tmp_error_count_oor = 0;
    uint16_t tmp_error_count_oh = 0;
    uint16_t tmp_error_count_rt = 0;
    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){
        g_BatteryPack.slave[idx_slave].temperatureFlags = 0;

        for(uint16_t idx_thermistor=0; idx_thermistor < TEMPERATURE_SENSORS_PER_SLAVE; idx_thermistor++){
            out_of_range = g_EventCount.slave[idx_slave].temperature_not_out_of_range[idx_thermistor] == 0;
            bad_value = g_EventCount.slave[idx_slave].temperature_value_ok[idx_thermistor] ==  0;
            not_read = g_EventCount.slave[idx_slave].temperature_read[idx_thermistor] == 0;

            #if HACK_USE_FILTER_TEMP
            if (tempDisable[TEMPERATURE_SENSORS_PER_SLAVE * idx_slave + idx_thermistor]) {
              continue;
            }
            #endif

            if (out_of_range){
                g_BatteryPack.slave[idx_slave].temperatureFlags |= (uint16_t)(1 << idx_thermistor);
                set_error_bit(ERROR_THERMISTOR_OUT_OF_RANGE);
                tmp_error_count_oor++;
                //total_slave_temp_problems++;
            }
            else if (bad_value){
                g_BatteryPack.slave[idx_slave].temperatureFlags |= (uint16_t)(1 << idx_thermistor);
                set_error_bit(ERROR_CELL_OVERHEAT);
                tmp_error_count_oh++;
                //total_slave_temp_problems++;
            }
            else if (not_read){
                has_temp_read_timeout = true;
                set_error_bit(ERROR_SLAVE_TEMPERATURE_READ_TIMEOUT);
                tmp_error_count_rt++;
            }
        }
    }

    //if(tmp_error_count_rt + tmp_error_count_oh + tmp_error_count_oor > skipErrors) {
    //    error_count++;
     //   if (tmp_error_count_oh > 0)set_error_bit(ERROR_CELL_OVERHEAT);
      //  if (tmp_error_count_oor > 0)set_error_bit(ERROR_THERMISTOR_OUT_OF_RANGE);
       // if (tmp_error_count_rt > 0) set_error_bit(ERROR_SLAVE_TEMPERATURE_READ_TIMEOUT);
   // }

    // Get the total pec fail count for statistics. Only increment the total pec fail count if there is no cv or temp timeout
    if (!has_cv_read_timeout && !has_temp_read_timeout) {
        uint16_t slave_pec_fail_count = 0;
        for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++) {
            slave_pec_fail_count += g_EventCount.slave[idx_slave].pec_error;
        }

        g_pecFailCount += slave_pec_fail_count;
    }

    _reset_event_count_slaves();

    return (error_count != 0);

}

void reset_all_g_EventCount(){
    _reset_event_count_current_probe();
    _reset_event_count_slaves();
}

bool check_interlock_no_error() {
    uint64_t intlck_error = (g_error_status >> 16) & 0x13BF; // TODO

    return intlck_error == 0;
}

bool _check_all_errors() {
    // HACK
    return 0 != ( g_error_status & 0x17BF00FE );
}

/**
 * @brief: Checks if there are any problems with EventCounts and sets fatal errors, killing the car.
 *
 * Does not handle interlock errors, as these are handled in check_interlock_circuit already.
 * (An interlock error already kills the car.)
 */
bool error_management(){
    static bool error_already_saved = 0;

    g_errorTimerMs = 0;
    _manage_error_current_probe();

    if (g_Time < 8) {
      return;
    }

    // Kill the car in case of an error in slave measurements.
    // The car might be killed by an IMD error or an AMS error in the main loop too
    if (_manage_error_slave_measurements()) {
      // HACK Timer before car kill
        openInterlock();
#ifndef USING_LAUNCHPAD
        if (error_already_saved==0){
            saveErrorStatus();
            error_already_saved = 1;
        }
#endif
    }

    return _check_all_errors();
}
