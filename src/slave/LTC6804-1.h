/*
 * LTC6804.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef LTC68041_H
#define LTC68041_H

#ifndef LTC6804_CS
#define LTC6804_CS QUIKEVAL_CS
#endif

#include <src/defines.h>

//---------------------------
// Default Configuration registers
//---------------------------
// ADC mode option
// 0 -> Selects Modes 27kHz, 7kHz or 26Hz with MD[1:0] Bits in ADC Conversion Commands (Default).
// 1 -> Selects Modes 14kHz, 3kHz or 2kHz with MD[1:0] Bits in ADC Conversion Commands.
// See ADC Mode for more information
#define ADCOPT 1 // if this value is changed, make sure to update timing values used to determine if action is done

// measurement times using ADC_MODE_FILTERED and ADCOPT=0. (2kHz)
#define MEAS_MS 8u // time taken for completion of CV measurement

// SWTEN Pin Status (Read Only)
// 0 -> SWTEN Pin at Logic 0
// 1 -> SWTEN Pin at Logic 1
#define SWTRD 0

// Reference Powered UP (if ref not powered up, IC needs to pause T_REFUP time before a measurement command.
// See operation state diagram.
// 0 -> Reference Shuts Down after Conversions (Default)
// 1 -> Reference Remains Powered Up Until Watchdog Timeout
#define REFON 1

// Undervoltage Comparison Voltage
// VUV = (Comparison_Voltage / 100µV / 16) -1
// Ex: for voltage of 4.2V: VUV = (4.2V / 100uV /16) -1 = 2624
#define VUV 2624 // 4.2V

// Overvoltage Comparison Voltage
// VOV = Comparison_Voltage / 100µV / 16
#define VOV 1563 // 2.5V

// Discharge cell
// DCC[x] = Discharge Cell.x (x in [1 to 12])
#define DCC_ALL_OFF 0x000

// DCTO = Discharge timeout in seconds
// when reading this value, represents time left (Ex: _60S means time_left = ]30sec to 60sec]
typedef enum {
    DCTO_OFF = 0x00U,
    DCTO_30S = 0x01U,
    DCTO_60S = 0x02U,
    DCTO_120S = 0x03U,
    DCTO_180S = 0x04U,
    DCTO_240S = 0x05U,
    DCTO_300S = 0x06U,
    DCTO_600S = 0x07U,
    DCTO_900S = 0x08U,
    DCTO_1200S = 0x09U,
    DCTO_1800S = 0x0AU,
    DCTO_2400S = 0x0BU,
    DCTO_3600S = 0x0CU,
    DCTO_4500S = 0x0DU,
    DCTO_5400S = 0x0EU,
    DCTO_7200S = 0x0FU,
} LTC6804_CellDischargeTimeout;


// GPIOx Pin Control Write:
#define GPIO_PULL_DOWN_ON 0
#define GPIO_PULL_DOWN_OFF 1 // (Default)
#define GPIO_ALL_PULL_DOWN_OFF 0x1F
#define GPIO_ALL_PULL_DOWN_ON 0x00
#define GPIO_1_MODE GPIO_PULL_DOWN_OFF

// Initial configuration
#define CFGR0_INIT configByte0(GPIO_ALL_PULL_DOWN_ON, REFON, SWTRD, ADCOPT)
#define CFGR1_INIT configByte1(VUV)
#define CFGR2_INIT configByte2(VOV, VUV)
#define CFGR3_INIT configByte3(VOV)
#define CFGR4_INIT configByte4(DCC_ALL_OFF)
#define CFGR5_INIT configByte5(DCTO_OFF, DCC_ALL_OFF)

//---------------------------
// Read Commands: Register Groups
//---------------------------
// Send read command and read back from SPI the register groups for every LTC in the chain
// List of internal memory register groups available for reading
typedef enum {
    READ_CONFIG = 0x0002,
    READ_CV_A   = 0x0004, // Cell Voltages 1, 2, 3
    READ_CV_B   = 0x0006, // Cell Voltages 4, 5, 6
    READ_CV_C   = 0x0008, // Cell Voltages 7, 8, 9
    READ_CV_D   = 0x000A, // Cell Voltages 10, 11, 12
    READ_AUX_A  = 0x000C, // GPIO measurements 1, 2, 3
    READ_AUX_B  = 0x000E, // GPIO measurements 4, 5 and REF (2nd reference voltage)
    READ_STAT_A = 0x0010, // SOC, ITMP, VA
    READ_STAT_B = 0x0012, // VD, Cells Overvoltages, Cells Undervoltages, THSD, MUXFAIL, RSVD, REV
    // READ_COMM      = 0x0722,
} LTC6804_ReadCommand;

//---------------------------
// Start Commands
//---------------------------
// Commands without read back
typedef enum {
    START_ADC_CV            = 0x0260, // ADCV
    /* START_ADC_CV            = 0x0270, // ADCV Allow discharge */
    START_ADC_GPIO          = 0x0460, // ADAX
    START_ADC_STATS         = 0x0468, // ADSTAT
    START_ADC_CV_AND_GPIO   = 0x046F, // ADCVAX
    /* START_ADC_CV_AND_GPIO   = 0x047F, // ADCVAX Allow discharge */

    START_OPEN_WIRE_ADC     = 0x0228, // ADOW
    START_SELF_TEST_CV      = 0x0207, // CVST
    START_SELF_TEST_GPIO    = 0x0407, // AXST
    START_SELF_TEST_STAT   = 0x040F, // STATST
    START_DIAGNOSE_MUX      = 0x0715, // DIAGN

    CLEAR_REG_CV            = 0x0711, // CLRCELL
    CLEAR_REG_AUX           = 0x0712, // CLRAUX
    CLEAR_REG_STAT          = 0x0713, // CLRSTAT

    // START_COMM                      = 0x0723, //STCOMM   //unused
} LTC6804_StartCommand;

//---------------------------
// Special Commands
//---------------------------
// Commands with read back or write more than a single command word
typedef enum {
    WRITE_CONFIG            = 0x0001, // WRCFG
    // POLL_ADC_DONE                = 0x714, // PLADC   // not available with daisy chained LTC6804-1
    // WRITE_COMM                      = 0x0721, // WRCOMM  //unused
} LTC6804_SpecialCommand;


// A group is a "register group". It consists of 6 bytes.
#define BYTES_IN_GROUP 6 // A register group is 6 registers of 8 bits per group (48bit total)
#define BYTES_IN_GROUP_WITH_PEC (BYTES_IN_GROUP+2) // LTC sends data + PEC (2 bytes). (6B+2B) * 8bit = 64bit total

#define WORDS_IN_GROUP 3 // MCU is on 16 bit word length
#define WORDS_IN_GROUP_WITH_PEC (WORDS_IN_GROUP+1)
#define WRCFG_CMD_LEN 4+(BYTES_IN_GROUP_WITH_PEC*TOTAL_SLAVES)

#define SPI_EXTRA_GARBAGE 1

typedef uint16_t LTC6804_Group[TOTAL_SLAVES][WORDS_IN_GROUP]; // contains one register group for each IC.
typedef uint16_t LTC6804_ConfigGroup[TOTAL_SLAVES][BYTES_IN_GROUP]; // contains config group (easier to work with bytes here)

/*!
 |CH | Dec  | Channels to convert |
 |---|------|---------------------|
 |000| 0    | All Cells           |
 |001| 1    | Cell 1 and Cell 7   |
 |010| 2    | Cell 2 and Cell 8   |
 |011| 3    | Cell 3 and Cell 9   |
 |100| 4    | Cell 4 and Cell 10  |
 |101| 5    | Cell 5 and Cell 11  |
 |110| 6    | Cell 6 and Cell 12  |
 */
typedef enum {
    CELL_CH_ALL     = 0,
    CELL_CH_1and7   = 1,
    CELL_CH_2and8   = 2,
    CELL_CH_3and9   = 3,
    CELL_CH_4and10  = 4,
    CELL_CH_5and11  = 5,
    CELL_CH_6and12  = 6,
} LTC6804_CellChannel;

/*!
 |CHG | Dec  |Channels to convert   |
 |----|------|----------------------|
 |000 | 0    | All GPIOS and 2nd Ref|
 |001 | 1    | GPIO 1               |
 |010 | 2    | GPIO 2               |
 |011 | 3    | GPIO 3               |
 |100 | 4    | GPIO 4               |
 |101 | 5    | GPIO 5               |
 |110 | 6    | Vref2                |
 */
typedef enum {
    AUX_CH_ALL   = 0, // GPIO 1-5 and VREF2
    AUX_CH_GPIO1 = 1,
    AUX_CH_GPIO2 = 2,
    AUX_CH_GPIO3 = 3,
    AUX_CH_GPIO4 = 4,
    AUX_CH_GPIO5 = 5,
    AUX_CH_VREF2 = 6,
} LTC6804_GPIOChannel;


typedef enum {
    STATUS_REG_ALL  = 0, // SOC, ITMP, VA, VD
    STATUS_REG_SOC  = 1,
    STATUS_REG_ITMP = 2,
    STATUS_REG_VA   = 3,
    // STATUS_REG_VD   = 4, // ADSTAT usage with VD is not recommended (see Datasheet: Data Acquisition System Diagnostics)
} LTC6804_StatusSelection;

/*!
 ADC Mode (depends on ADCOPT)

 |MD| Dec  | ADC Conversion Model|
 |--|------|---------------------|
 |01| 1    | Fast                |
 |10| 2    | Normal              |
 |11| 3    | Filtered            |
 */
typedef enum {
    ADC_MODE_FAST   = 0x01,
    ADC_MODE_NORMAL = 0x02,
    ADC_MODE_FILTERED   = 0x03,
} LTC6804_ADCMode;


// Self Test Mode. See table 10 in datasheet. Used with CVST, AXST and STATST
typedef enum {
    SELF_TEST_1 = 0x01,
    SELF_TEST_2 = 0x02,
} LTC6804_SelfTest;

/*!****************************************************
 \brief Controls if Discharging transitors are enabled
 or disabled during Cell conversions.

 |DCP | Discharge Permitted During conversion  |
 |----|----------------------------------------|
 |0   | No - discharge is not permitted        |
 |1   | Yes - discharge is permitted           |

 ********************************************************/
typedef enum {
    DISCHARGE_NOT_PERMITTED = 0,
    DISCHARGE_PERMITTED  = 1,
} LTC6804_DischargePermission;

extern uint16_t open_wire_cell_number; // Problematic cell number

/*!
 * Config data to write on device
 */
extern LTC6804_ConfigGroup g_slave_config;
/*
 Pre computed crc15 table used for the LTC6804 PEC calculation

 The code used to generate the crc15 table is:

 void generate_crc15_table()
 {
 int remainder;
 for(int i = 0; i<256;i++)
 {
 remainder =  i<< 7;
 for (int bit = 8; bit > 0; --bit)
 {

 if ((remainder & 0x4000) > 0)//equivalent to remainder & 2^14 simply check for MSB
 {
 remainder = ((remainder << 1)) ;
 remainder = (remainder ^ 0x4599);
 }
 else
 {
 remainder = ((remainder << 1));
 }
 }

 crc15Table[i] = remainder&0xFFFF;

 }
 }
 */

static const unsigned int crc15Table[256] = {
        0x0,
        0xc599,
        0xceab,
        0xb32,
        0xd8cf,
        0x1d56,
        0x1664,
        0xd3fd,
        0xf407,
        0x319e,
        0x3aac,  //!<precomputed CRC15 Table
        0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3, 0x7e6a, 0x5990,
        0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a,
        0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76,
        0xebef, 0xe0dd, 0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c, 0x3d6e,
        0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d, 0x1a94, 0x11a6, 0xd43f, 0x5e52,
        0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf, 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31,
        0x79a8, 0xa8eb, 0x6d72, 0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423,
        0x41ba, 0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 0x34e2, 0x3fd0,
        0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614,
        0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070, 0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0,
        0xdc79, 0xfb83, 0x3e1a, 0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2,
        0xe46b, 0xef59, 0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
        0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350, 0x51d6,
        0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5,
        0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089,
        0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453, 0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054,
        0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368,
        0x96f1, 0x9dc3, 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095 };


static inline uint16_t configByte0(uint16_t GPIOPullDownOFF, uint16_t refon, uint16_t swtrd, uint16_t adcopt) {
    uint16_t config;

    refon = refon & 0x01;
    swtrd = swtrd & 0x01;
    adcopt = adcopt & 0x01;

    config = (GPIOPullDownOFF << 3) | (refon << 2) | (swtrd << 1) | (adcopt & 0x01);
    return (config & 0x00FF);
}

static inline uint16_t configByte1(uint16_t vuv) {
    uint16_t config;

    vuv = vuv & 0x00FF;

    config = vuv;
    return (config & 0x00FF);
}

static inline uint16_t configByte2(uint16_t vov, uint16_t vuv) {
    uint16_t config;

    vuv = vuv & 0x0F00;
    vov = vov & 0x000F;

    config = ((vov << 4) & 0xF0) | ((vuv >> 8) & 0x0F);
    return (config & 0x00FF);
}

static inline uint16_t configByte3(uint16_t vov) {
    uint16_t config;

    vov = vov & 0x0FF0;
    config = vov >> 4;

    return (config & 0x00FF);
}

static inline uint16_t configByte4(uint16_t dcc) {
    // DCC[x] = Discharge Cell number x (x in [1 to 12]) Ex: 0x01 is cell #1 set to discharge.
    uint16_t config;

    dcc = dcc & 0x00FF;

    config = dcc;
    return (config & 0x00FF);
}

static inline uint16_t configByte5(LTC6804_CellDischargeTimeout dcto, uint16_t dcc) {
    // DCTO[x] = Discharge Cell number x (x in [1 to 12]) Ex: 0x01 is cell #1 set to discharge.
    // DCC[x] = Discharge Cell number x (x in [1 to 12]) Ex: 0x01 is cell #1 set to discharge.
    uint16_t config;

    dcc = dcc & 0x0F00;

    config = (dcto<<4) | (dcc >> 8);
    return (config & 0x00FF);
}

/**
 * @brief Formats the start Cell Voltage ADC measurement command ADCV
 */
static inline uint16_t format_command_ADCV(LTC6804_ADCMode adc_mode, uint16_t discharge_permitted, LTC6804_CellChannel channel) {
    // adc_mode = ADC Mode (depends on ADCOPT)
    // discharge_permitted = Discharge Permitted (DCP_DISABLED or DCP_ENABLED)
    // channel = Cell Selection for ADC Conversion
    return (START_ADC_CV) | (adc_mode<<7) | (discharge_permitted << 4) | (channel);
}

/**
 * @brief Formats the start open wire ADC measurement command ADOW
 */
static inline uint16_t format_command_ADOW(LTC6804_ADCMode adc_mode, LTC6804_DischargePermission discharge_mode, LTC6804_CellChannel channel, uint16_t pup) {
    // discharge_permitted = Discharge Permitted (DCP_DISABLED or DCP_ENABLED)
    // pup = pull-up/pull-down (1 is Pull-Up, 0 is Pull-Down)
    return (START_OPEN_WIRE_ADC) | (adc_mode<<7) | (pup<<6) |(discharge_mode << 4) | (channel);
}

/**
 * @brief Formats a start self test command CVST, AXST or STATST
 */
static inline uint16_t format_command_self_test(LTC6804_StartCommand command, LTC6804_ADCMode adc_mode, LTC6804_SelfTest self_test_mode){
    // used for
    return (command) | (adc_mode<<7) | (self_test_mode<<5);
}

/**
 * @brief Formats the start GPIO ADC measurement command ADAX
 */
static inline uint16_t format_command_ADAX(LTC6804_ADCMode adc_mode, LTC6804_GPIOChannel channel) {
    return (START_ADC_GPIO) | (adc_mode<<7) | (channel);
}

/**
 * @brief Formats the start the Status (STAT group) measurement command ADSTAT
 */
static inline uint16_t format_command_ADSTAT(LTC6804_ADCMode adc_mode, LTC6804_StatusSelection channel) {
    return (START_ADC_STATS) | (adc_mode<<7) | (channel);
}

/**
 * @brief Formats the start Status (STAT) self test STATST
 */
static inline uint16_t format_command_STATST(LTC6804_ADCMode adc_mode, LTC6804_SelfTest self_test_mode) {
    return (START_SELF_TEST_STAT) | (adc_mode<<7) | (self_test_mode<<5);
}

/**
 * @brief Formats the start CV and GPIO combined measurements command ADCVAX
 */
static inline uint16_t format_command_ADCVAX(LTC6804_ADCMode adc_mode, LTC6804_DischargePermission discharge_mode) {
    return (START_ADC_CV_AND_GPIO) | (adc_mode<<7) | (discharge_mode << 4);
}

void slave_initialize();
ErrorFlag LTC6804_send_read_command(uint16_t read_command, LTC6804_Group *data);
void LTC6804_send_start_command(uint16_t command);
void LTC6804_write_config(LTC6804_ConfigGroup *config);
void LTC6804_start_next_temperature_sensor(uint16_t mux_GPIO_selection);
void LTC6804_remove_all_cell_discharge_config();

ErrorFlag spi_read_group(LTC6804_Group *data);
void spi_write_array(uint16_t len, uint16_t *data);
uint16_t pec15_calc(uint16_t len, uint16_t *data);

void wakeup_idle();
void wakeup_sleep();

#endif
