/*
 * main.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_DEFINES_H_
#define SRC_DEFINES_H_

#include <device.h>

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

// Error codes for Master BMS and Slaves
typedef enum {
    SUCCESS,
    FAIL,
    ERROR_BAD_PEC,
    ERROR_DIAGN_MUX, // LTC6804's internal MUX problem.
} ErrorFlag;

#define DEBUG 1 // MUST BE 0 IN PRODUCTION
#define NO_SLAVE_TEST 0 // MUST BE 0 IN PRODUCTION
#define DISABLE_FAN_OK 1 // MUST BE 0 IN PRODUCTION

#if (DEBUG ==1)
#warning Debug mode
#define WATCHDOG_MODE_INTERRUPT 1 // 0 for reset, 1 for interrupts
#elif (DEBUG == 0)
#define WATCHDOG_MODE_INTERRUPT 0
#endif

// Define this if you are willing to test CAN using the internal can test feature of the C2000
#define CAN_INTERNAL_DEBUG  1

#define ADC_RESOLUTION 12
#define CURRENT_PROBE_SAMPLING_FREQ 20000 // probe must be sampled faster than 15kHz

#define TOTAL_SEGMENTS      5 // NOTE: Adjust per the number of segment that is used
#define SLAVE_PER_SEGMENT   2

#define TOTAL_SLAVES        SLAVE_PER_SEGMENT*TOTAL_SEGMENTS // Number of slaves (2x IC per board)
#define CELL_VOLTAGES_PER_SLAVE 10

#define HACK_USE_FILTER_TEMP 1

/*
 * Temperature Mux selection from GPIO: this value cycles through [0 to 15]
 * There are 16 sensors on every slave
 * Global variable
 */
extern uint16_t g_temperatureSensorIdx;
#define TEMPERATURE_SENSORS_PER_SLAVE 16

// CHARGER DEFINES
#define DEFAULT_CHARGE_CV_COMPENSATION_OFFSET       500 // [100uV] Offset applied to nominal battery pack cell voltage at which the max current output by the charger begins to be limited 
#define MAX_CHARGER_CURRENT                         30  //[0.1A] 3A on 115Vac, 7A on 230Vac NOTE : change value according to outlet 
#define MIN_CHARGER_CURRENT                         5   //[0.1A] Minimal current before charging stops
#define MAX_CHARGER_VOLTAGE                         4000 // [0.1V] Maximum voltage for the battery pack
#define DEFAULT_CHARGE_BP_START_OFFSET              400000 // [100uV] Offset applied to nominal battery pack voltage at which we will start charging 

/**
 * Contains a BMS slave data (LTC6804)
 */
typedef struct {
    uint16_t cv[CELL_VOLTAGES_PER_SLAVE];
    uint16_t temperature[TEMPERATURE_SENSORS_PER_SLAVE];
    uint16_t averageCv;
    uint16_t cellDischargeON; // lsb is cell 0
    uint16_t voltageFlags; // lsb is cell 0
    uint16_t temperatureFlags;
    float internalDieTemperature; // in °C. MAX is 150
    bool muxfail;
    bool internalDieThermalShutdown;
    uint16_t balancingJustFinish;
    uint16_t vd; // Digital power supply measurement (VREGD) = VD * 100µV
    uint16_t vref2; // REF (2nd reference voltage)
} Slave;


/**
 * Counts actions performed by a slave. Contained in BMSEventCount.
 *
 * Helps to debug and determine percentage of actions failed.
 */
typedef struct{

    uint16_t cv_A_read; // read register success
    uint16_t cv_B_read;
    uint16_t cv_C_read;
    uint16_t cv_D_read;
    uint16_t cv_not_out_of_range[CELL_VOLTAGES_PER_SLAVE]; // cv value is ok
    uint16_t cv_not_UVOV[CELL_VOLTAGES_PER_SLAVE];

    uint16_t temperature_read[TEMPERATURE_SENSORS_PER_SLAVE];
    uint16_t temperature_not_out_of_range[TEMPERATURE_SENSORS_PER_SLAVE];
    uint16_t temperature_value_ok[TEMPERATURE_SENSORS_PER_SLAVE]; // ERROR_CELL_OVERHEAT

    uint16_t open_wire_error[CELL_VOLTAGES_PER_SLAVE];
    uint16_t pec_error;
}SlaveEventCount;

/**
 * Counts actions performed by the BMS.
 *
 * Helps to debug and determine percentage of actions failed.
 */
struct EventCount
{
    SlaveEventCount slave[TOTAL_SLAVES];
    uint16_t slave_pec_total;
    uint16_t slave_cv_total;

    volatile uint16_t current_probe_total;
    volatile uint16_t current_probe_not_out_of_range;
    volatile uint16_t current_probe_not_overcurrent;
    volatile uint16_t current_probe_ref_ok;

    uint16_t main_loop;
};
extern struct EventCount g_EventCount;

/*
 * Globals used to measure time between actions
 */
extern volatile uint16_t g_slaveMeasurementCountMs;
extern volatile uint16_t g_saveDataEepromSeconds;
extern uint16_t g_pecFailCount;
extern volatile uint16_t g_errorTimerMs;

struct MasterBMS
{
    uint16_t pwr_1V2;
    uint16_t pwr_3V3;
    uint16_t pwr_5V;
    uint16_t current_probe_ref;
    uint16_t imd_data_out;
    uint16_t temperature;
    uint16_t hv_measurement;
    bool interlock_no_error;
    bool charger_connected;
};
extern struct MasterBMS g_masterBMS;

extern int16_t g_current_probe_offset;
extern bool g_close_interlock_after_next_valid_error_check;

/**
 * Elcon CAN Charger state
 *
 * All related to external charger
 *
 */
struct Charger
{
    uint16_t status; // 0: charger is open and on charge. 1: Battery protection, the charger closes its output
    uint16_t voltage; // [0.1V]
    uint16_t current; // [0.1A]
};
extern struct Charger g_Charger;

/**
 * Error definitions
 */
typedef enum{
    NO_ERROR = 0,
    ERROR_CV_OUT_OF_RANGE, // cv not in posible range of values
    ERROR_CV_UVOV, // under or over voltage (only set if in possible CV range
    ERROR_THERMISTOR_OUT_OF_RANGE,

    ERROR_CELL_OVERHEAT,
    ERROR_SLAVE_CV_READ_TIMEOUT, // No valid PEC 500ms
    ERROR_SLAVE_TEMPERATURE_READ_TIMEOUT, // No valid PEC 500ms
    ERROR_SLAVE_THERMAL_SHUTDOWN, // thermal shutdown from a slave

    ERROR_SLAVE_MUXFAIL, // internal mux failure
    ERROR_SLAVE_OPEN_WIRE, // open wire check detected an open connection 
    ERROR_SLAVE_ACCURACY, // accuracy check with VREF2: System is out of specified tolerance
    ERROR_SLAVE_SELF_TEST_CV,   // failed self test CVST

    ERROR_SLAVE_SELF_TEST_GPIO, // failed self test GPST
    ERROR_SLAVE_SELF_TEST_STAT, // failed self test STATST
    ERROR_SLAVE_CLEAR_TEST, // test clear functions failed
    ERROR_FAN_SHOULD_BE_RUNNING, 

    ERROR_INTERLOCK_24V, // MCU_24V_INTLCK not ok
    ERROR_INTERLOCK_IMD_OPEN, // MCU_IMD_OK (17)
    ERROR_INTERLOCK_IMD_NOT_CLOSED, // AMS does not confirm read IMD status
    ERROR_INTERLOCK_IMD_NOT_OPEN, // AMS does not confirm read IMD status

    ERROR_INTERLOCK_AMS_OPEN, // AMS is open (20)
    ERROR_INTERLOCK_AMS_NOT_OPEN, // Interlock / car not killed when AMS is open
    ERROR_INTERLOCK_AMS_NOT_CLOSED, // AMS should be closed but is not
    ERROR_INTERLOCK_HVD, // HVD does not work

    ERROR_INTERLOCK_HVD_IN_SHOULD_NOT_WORK, // HVD_OUT is ok but HVD_IN is not. (HVD works when it shouldn't)
    ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_1,
    ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_1_SHOULD_NOT_WORK,
    ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_2, // TSMS

    ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_2_SHOULD_NOT_WORK,
    ERROR_CURRENT_ADC_INTERRUPT_FCT_NOT_CALLED,
    ERROR_CURRENT_PROBE_OUT_OF_RANGE, // current probe range of values is 0V to 3V == -450A to 450A
    ERROR_CURRENT_PROBE_OVERCURRENT, //

    ERROR_CURRENT_PROBE_REF_NOT_OK, // problem with current probe reference

    ERROR_TEMP_OVER_60C,
} ERROR_BMS;
extern uint64_t g_error_status;
inline void set_error_bit(ERROR_BMS e) {g_error_status |= ((uint64_t)1 << e);} 
inline void clear_error_bit(ERROR_BMS e) {g_error_status &= ~((uint64_t)1<<e);}
inline bool get_error_bit(ERROR_BMS e) {return g_error_status & ((uint64_t)1 << e);}

#define CELL_DISCHARGE_TIME DCTO_30S

/**
 * Info related to data save
 */
#define SECONDS_BETWEEN_DATA_SAVE 10 // This allows 400 days of writing before failure

#define EEPROM_ADDR_ERROR_STATUS (256*0)
#define EEPROM_ADDR_STATE_OF_CHARGE (256*1)
#define EEPROM_ADDR_PARAM (256*2)



/**
 * GPIO definitions: pin mapping and functions
 */
// Debug LED
#define GPIO_MCU_DEBUG_LED_0        18U
#define PIN_MAP_MCU_DEBUG_LED_0     GPIO_18_GPIO18
#define GPIO_MCU_DEBUG_LED_1        19U
#define PIN_MAP_MCU_DEBUG_LED_1     GPIO_19_GPIO19
#define GPIO_MCU_DEBUG_LED_2        92U
#define PIN_MAP_MCU_DEBUG_LED_2     GPIO_92_GPIO92

// Fan control
#define GPIO_PWM_MCU_FAN_CONTROL    2U
// GPIO_2_GPIO2
#define PIN_MAP_PWM_MCU_FAN_CONTROL GPIO_2_EPWM2A
#define PWM_MCU_FAN_BASE            EPWM2_BASE

// Fan error check
#define GPIO_MCU_FAN_NOT_RUNNING    69U
#define PIN_MAP_MCU_FAN_NOT_RUNNING GPIO_69_GPIO69

// Pump control
#define GPIO_MCU_PUMP_CONTROL       10U
#define PIN_MAP_MCU_PUMP_CONTROL    GPIO_10_GPIO10

// Charger
#define GPIO_MCU_CHARGER_DETECT     15U
#define PIN_MAP_MCU_CHARGER_DETECT  GPIO_15_GPIO15

// Over Current Detection Probe
#define GPIO_MCU_OCD_PROBE          16U
#define PIN_MAP_MCU_OCD_PROBE       GPIO_16_GPIO16

// Receives a specific duty cycle which represents the IMD detailed status
#define GPIO_MCU_DATA_OUT_HIGH      17U
#define PIN_MAP_MCU_DATA_OUT_HIGH   GPIO_17_GPIO17

// Tractive System
#define GPIO_MCU_TS_DISABLED          20U
#define PIN_MAP_MCU_TS_DISABLED       GPIO_20_GPIO20

// EEPROM
#define GPIO_EEPROM_SPI_MOSI        58U
#define PIN_MAP_EEPROM_SPI_MOSI     GPIO_58_SPISIMOA
#define GPIO_EEPROM_SPI_MISO        59U
#define PIN_MAP_EEPROM_SPI_MISO     GPIO_59_SPISOMIA
#define GPIO_EEPROM_SPI_SCK         60U
#define PIN_MAP_EEPROM_SPI_SCK      GPIO_60_SPICLKA
#define GPIO_EEPROM_SPI_CS          61U
#define PIN_MAP_EEPROM_SPI_CS       GPIO_61_SPISTEA
#define EEPROM_SPI_BASE             SPIA_BASE

// SPI (to ISOspi to LTC6804)
#define GPIO_MCU_SPI_MOSI           63U
#define PIN_MAP_MCU_SPI_MOSI        GPIO_63_SPISIMOB
#define GPIO_MCU_SPI_MISO           64U
#define PIN_MAP_MCU_SPI_MISO        GPIO_64_SPISOMIB
#define GPIO_MCU_SPI_SCK            65U
#define PIN_MAP_MCU_SPI_SCK         GPIO_65_SPICLKB
#define GPIO_MCU_SPI_CS             66U
#define PIN_MAP_MCU_SPI_CS          GPIO_66_SPISTEB
#define MCU_SPI_BASE                SPIB_BASE

// CAN
#define GPIO_MCU_CAN_TX             12U
#define PIN_MAP_MCU_CAN_TX          GPIO_12_CANTXB
#define GPIO_MCU_CAN_RX             13U
#define PIN_MAP_MCU_CAN_RX          GPIO_13_CANRXB
#define MCU_CAN_BASE                CANB_BASE
#define MCU_CAN_FREQ                250000 // 1000 kbps
#define MCU_CHARGER_CAN_FREQ        250000 // 1Mbps
#define MCU_CAN_INT                 INT_CANB0

// CAN reprog
#define GPIO_MCU_CAN_RX_REPROG      62U
#define PIN_MAP_MCU_CAN_RX_REPROG   GPIO_62_CANRXA
#define GPIO_MCU_CAN_TX_REPROG      71U
#define PIN_MAP_MCU_CAN_TX_REPROG   GPIO_71_CANTXA

// UART (SCI)
#define GPIO_MCU_UART_TX            89U
#define PIN_MAP_MCU_UART_TX         GPIO_89_SCITXDC
#define GPIO_MCU_UART_RX            90U
#define PIN_MAP_MCU_UART_RX         GPIO_90_SCIRXDC
#define UART_SCI_BASE               SCIC_BASE

// Interlock output:
// AMS control, (kill switch). Car is killed when pin is 0.
#define GPIO_MCU_INTLCK_AMS         11U
#define PIN_MAP_MCU_INTLCK_AMS      GPIO_11_GPIO11
// Interlock status/check inputs. 1 if OK.
#define GPIO_MCU_24V_INTLCK         70U
#define PIN_MAP_MCU_24V_INTLCK      GPIO_70_GPIO70
#define GPIO_MCU_IMD_OK             78U
#define PIN_MAP_MCU_IMD_OK          GPIO_78_GPIO78
#define GPIO_MCU_INTLCK_AMS_IN      43U
#define PIN_MAP_MCU_INTLCK_AMS_IN   GPIO_43_GPIO43
#define GPIO_MCU_INTLCK_HVD_OUT     41U
#define PIN_MAP_MCU_INTLCK_HVD_OUT  GPIO_41_GPIO41
#define GPIO_MCU_INTLCK_HVD_IN      42U
#define PIN_MAP_MCU_INTLCK_HVD_IN   GPIO_42_GPIO42
#define GPIO_MCU_INTLCK_IN_2        86U
#define PIN_MAP_MCU_INTLCK_IN_2     GPIO_86_GPIO86
#define GPIO_MCU_INTLCK_OUT_2       99U
#define PIN_MAP_MCU_INTLCK_OUT_2    GPIO_99_GPIO99

// MCU clock out
#define GPIO_MCU_XLKOUT             73U
#define PIN_MAP_MCU_XLKOUT          GPIO_73_XCLKOUT

// TSAL
// XXX add in GPIO init function. Inputs or Outputs? (Seems to work actually)
#define GPIO_MCU_TSAL_60V_OFF           3U
#define PIN_MAP_MCU_TSAL_60V_OFF        GPIO_3_GPIO3
#define GPIO_MCU_TSAL_AIR_MINUS_OFF     14U
#define PIN_MAP_MCU_TSAL_AIR_MINUS_OFF  GPIO_14_GPIO14
#define GPIO_MCU_TSAL_PRECHARGE_OFF     21U
#define PIN_MAP_MCU_TSAL_PRECHARGE_OFF  GPIO_21_GPIO21
#define GPIO_MCU_TSAL_AIR_PLUS_OFF      87U
#define PIN_MAP_MCU_TSAL_AIR_PLUS_OFF   GPIO_87_GPIO87



// SCI reprog
//#define GPIO_MCU_SCI_RX_3V3         85U
//#define PIN_MAP_MCU_SCI_RX_3V3      GPIO_85_SCIRXDA

/*
 * ADC measurement pins
 */
// ADC_A
#define MCU_REFERENCE_CURRENT_PROBE_ADC_CH ADC_CH_ADCIN4 // used to debug. Should be 2.5V
#define MCU_REFERENCE_CURRENT_PROBE_ADC_SOC ADC_SOC_NUMBER4
#define MCU_DATA_OUT_HIGH_FILTERED_CH ADC_CH_ADCIN3 // average of a PWM with specific duty cycle
#define MCU_DATA_OUT_HIGH_FILTERED_SOC ADC_SOC_NUMBER3
#define MCU_TEMP_MEASUREMENT_ADC_CH ADC_CH_ADCIN2 // thermistance, ambiant temperature
#define MCU_TEMP_MEASUREMENT_ADC_SOC ADC_SOC_NUMBER2
#define MCU_HV_MEASUREMENT_ADC_CH ADC_CH_ADCIN1 // Hgh Voltage
#define MCU_HV_MEASUREMENT_ADC_SOC ADC_SOC_NUMBER1
#define MCU_CURRENT_PROBE_ADC_CH ADC_CH_ADCIN0 // current probe
#define MCU_CURRENT_PROBE_ADC_SOC ADC_SOC_NUMBER0

// ADC_B
// Measure every second or less:
#define MCU_1V2_CH ADC_CH_ADCIN0
#define MCU_1V2_SOC ADC_SOC_NUMBER0
// received value must be multiplied by 2
#define MCU_3V3_CH ADC_CH_ADCIN1
#define MCU_3V3_SOC ADC_SOC_NUMBER1
// received value must be multiplied by 2
#define MCU_5V_CH ADC_CH_ADCIN2
#define MCU_5V_SOC ADC_SOC_NUMBER2


#if MCU_SPI_BASE == EEPROM_SPI_BASE
#error "SPI_BASE used twice!"
#endif


#include "globals.h"
#endif /* SRC_DEFINES_H_ */
