// BMS Master
//
// Global variables and their initialisation
//
#ifndef _SRC_GLOBALS_H
#define _SRC_GLOBALS_H

#include <device.h>
#include "defines.h"


struct threading_t {
    uint16_t fanStartDelay; // Between 0-95 9V, 64-255 progressive.
};
extern struct threading_t threading;

/**
 * Contains BMS Parameters that can be modified from CAN messages.
 * Loaded and saved in EEPROM.
 */
struct ParametersBMS
{
    // all in 100uV
    uint16_t cv_discharge_max;
    uint16_t cv_discharge_min;
    uint16_t cv_charge_max; // do not charge if one cell is above this value
    uint16_t cv_charge_min; // do not charge if one cell is below this value
    uint16_t cv_range_max; // implausible measurement below this value
    uint16_t cv_range_min; // implausible measurement below this value
    uint16_t cv_balance_min_threshold; // starts balancing when one cell is above this value
    uint16_t cv_average_stop_charge; // stops charging when average is above this value

    // in seconds
    uint16_t charge_time_max;

    uint16_t charge_cycle_total_count; //battery_health parameter: Approximation of Number of battery discharge cycles

    // in Coulombs
    // SOC is 100 when coulomb_counter is coulomb_count_max
    uint16_t coulomb_count_max; // SOC = 100%. coulomb counter resets to this value.
    uint16_t coulomb_count_min; // SOC = 0%.

    // in 100uV
    // voltage gets smaller with higher temperature (MAX and MIN are counterintuitive)
    // pass variables through celsius_to_thermistance before writing here
    uint16_t thermistor_min; // max temperature tolerated before raising error
    uint16_t thermistor_min_charging; // max while charging
    uint16_t thermistor_range_min;
    uint16_t thermistor_range_max;
    uint16_t thermistor_fan_start;
    uint16_t thermistor_fan_stop;

    // ms
    uint16_t error_check_ms; // time between each error_checks
};
extern struct ParametersBMS parameters;
// XXX refactor
#define g_Param parameters

/**
 * Main data storage for BMS States
 * Contains links to slaves data
 */
struct BatteryPack
{
    Slave slave[TOTAL_SLAVES];

    // Units are set to fit with that ADC measurement directly (see util.c conversion functions for examples)
    // Units are in [12.08 μC] of charge.
    // Which equals [(450Ampere /1862) * (1second/CURRENT_PROBE_SAMPLING_FREQ)] See convert_current_probe_to_mAh and other conversion fct for more info
    volatile int64_t coulomb_counter; // Pack charge.
    volatile int16_t total_current; // latest current probe measurement. [-1872 to 1872] is [-450A to 450A]
    volatile uint32_t total_voltage; // comes from total of all cells read by slaves

    uint16_t averageCv; // average of slaves averages

    uint16_t max_cv; // in 100uV
    uint16_t min_cv; // in 100uV

    uint16_t max_thermistor; // in 100uV
    uint16_t min_thermistor; // in 100uV

    // Internal State controlled by our device
    bool charging;
};
extern struct BatteryPack batteryPack;
// XXX Refactor
#define g_BatteryPack batteryPack

/**
 * Raw ADC values will be stored here and will be processed later not in ISR
 */
struct RawAdcValues
{
    volatile uint16_t pwr_3v3;
    volatile uint16_t pwr_5v;
    volatile uint16_t pwr_1v2;
    volatile uint16_t current_probe;
    volatile uint16_t current_probe_ref;
    volatile uint16_t imd_data_out;
    volatile uint16_t master_temperature;
    volatile uint16_t hv_measurement;
    volatile bool process_groupA_ready;
    volatile bool process_groupB_ready;
};
extern struct RawAdcValues rawAdcValues;
// XXX Refac
#define g_RawAdcValues rawAdcValues

/**
 * Rtc counter updated every second
 */
extern uint32_t time;
// XXX Refactor
#define g_Time time

struct ErrorCVTimer {
    uint32_t oor;
    uint32_t ovuv;
};
#define CV_DELAY_ERROR 10 // s
extern struct ErrorCVTimer errorCVTimer;


  //////////////////
 /// Operations ///
//////////////////

/**
 * (Re)Initialise global variables
 */
void initializeGlobals(void);

#define G_START_FAN() threading.fanStartDelay = MAX(1, threading.fanStartDelay)
#define G_STOP_FAN() threading.fanStartDelay = 0

#endif /* _SRC_GLOBALS_H */
