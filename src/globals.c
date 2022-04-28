// BMS Master
//
// Global variables and their initialisation
//
#include "globals.h"

  /////////////////////
 /// Configuration ///
/////////////////////

/**
 * Default parameters if none were received by CAN or read in EEPROM.
 *
 * See ParametersBMS attributes for info on values and units
 */
// voltages related to cell_voltages. in 100uV
#define DEFAULT_CV_DISCHARGE_MAX_ACCEPTABLE 42500
#define DEFAULT_CV_DISCHARGE_MIN_ACCEPTABLE 25000
#define DEFAULT_CV_CHARGE_MAX 42000
#define DEFAULT_CV_CHARGE_MIN 30000
#define DEFAULT_CV_RANGE_MAX 50000 // [100μV] Max ADC value from LTC before an overflow is thrown
#define DEFAULT_CV_RANGE_MIN 20000 // 2V is Cell cutoff voltage
#define DEFAULT_CV_BALANCE_MIN_VOLTAGE_THRESHOLD 36000 // NOTE Originally 40000
#define DEFAULT_CV_AVERAGE_STOP_CHARGE 42000

#define DEFAULT_CHARGE_TIME_MAX (8*3600)

#define DEFAULT_CHARGE_CYCLE_TOTAL_COUNT 0 // NOTE update after a couple of battery cycles

#define DEFAULT_COULOMB_COUNT_MAX 64800 // theoretical max is 3Ah * 6 * 3600 Max value before an overflow is thrown
#define DEFAULT_COULOMB_COUNT_MIN 8640 // (3Ah - 2.6Ah) * 6 * 3600

// see for values https://docs.google.com/spreadsheets/d/1sNG3cwA7LgruL5ftnRLfCp38skVUsABsJqtSGXZg6QY/edit?usp=sharing
// or use imprecise fct: temperature_celsius = -48.560832 * temperature_voltage  + 94.049611;
// or see celsius_to_thermistance_adc_unit
#define DEFAULT_THERMISTOR_MIN 3099 // convertCelsiusToThermistance(59)
#define DEFAULT_THERMISTOR_MIN_CHARGING 3099 // convertCelsiusToThermistance(59)
#define DEFAULT_THERMISTOR_RANGE_MAX 24300 // -10 Celsius
#define DEFAULT_THERMISTOR_RANGE_MIN 2660 // 100 Celsius
#define DEFAULT_THERMISTOR_FAN_STOP convertCelsiusToThermistance(30)
#define DEFAULT_THERMISTOR_FAN_START convertCelsiusToThermistance(35)

#define DEFAULT_ERROR_CHECK_MS 2000


  ////////////////////
 /// Declarations ///
////////////////////

struct ParametersBMS parameters;
struct BatteryPack batteryPack;
struct RawAdcValues rawAdcValues;
struct threading_t threading;
struct ErrorCVTimer errorCVTimer;

uint32_t time;

  //////////////
 // Helpers ///
//////////////

/**
 * Initializes the Parameter structure, before reading the structure from EEPROM
 * Any value set here might be overwritten when reading values from EEPROM on startup.
 */
static inline void initDefaultParam() {
    parameters.cv_discharge_max = DEFAULT_CV_DISCHARGE_MAX_ACCEPTABLE;
    parameters.cv_discharge_min = DEFAULT_CV_DISCHARGE_MIN_ACCEPTABLE;
    parameters.cv_charge_max = DEFAULT_CV_CHARGE_MAX;
    parameters.cv_charge_min = DEFAULT_CV_CHARGE_MIN;
    parameters.cv_range_max = DEFAULT_CV_RANGE_MAX;
    parameters.cv_range_min = DEFAULT_CV_RANGE_MIN;
    parameters.charge_time_max = DEFAULT_CHARGE_TIME_MAX;
    parameters.cv_balance_min_threshold = DEFAULT_CV_BALANCE_MIN_VOLTAGE_THRESHOLD;
    parameters.cv_average_stop_charge = 41800; //DEFAULT_CV_AVERAGE_STOP_CHARGE;

    parameters.charge_time_max = DEFAULT_CHARGE_TIME_MAX;

    parameters.charge_cycle_total_count = DEFAULT_CHARGE_CYCLE_TOTAL_COUNT;

    parameters.coulomb_count_max = DEFAULT_COULOMB_COUNT_MAX;
    parameters.coulomb_count_min = DEFAULT_COULOMB_COUNT_MIN;

    parameters.thermistor_min = DEFAULT_THERMISTOR_MIN;
    parameters.thermistor_min_charging = DEFAULT_THERMISTOR_MIN_CHARGING;
    parameters.thermistor_range_min = DEFAULT_THERMISTOR_RANGE_MIN;
    parameters.thermistor_range_max = DEFAULT_THERMISTOR_RANGE_MAX;
    parameters.thermistor_fan_start = DEFAULT_THERMISTOR_FAN_START;
    parameters.thermistor_fan_stop = DEFAULT_THERMISTOR_FAN_STOP;

    parameters.error_check_ms = DEFAULT_ERROR_CHECK_MS;
}

static inline void initDefaultBatteryPack() {
    batteryPack.max_cv = 0;
    batteryPack.max_thermistor = 0;
    batteryPack.min_thermistor = 0;
    batteryPack.charging = 0;
    batteryPack.coulomb_counter = 0; // THIS SHOULD NOT BE 0 AFTER READING FROM EEPROM
}

static inline void initDefaultRawAdcValues() {
    rawAdcValues.current_probe = 0;
    rawAdcValues.current_probe_ref = 0;
    rawAdcValues.hv_measurement = 0;
    rawAdcValues.imd_data_out = 0;
    rawAdcValues.master_temperature = 0;
    rawAdcValues.pwr_1v2 = 0;
    rawAdcValues.pwr_3v3 = 0;
    rawAdcValues.pwr_5v = 0;
    rawAdcValues.process_groupA_ready = false;
    rawAdcValues.process_groupB_ready = false;
}


/////////////////////
/// Implementaton ///
/////////////////////

void initializeGlobals() {
    initDefaultBatteryPack();
    initDefaultParam();
    initDefaultRawAdcValues();
}
