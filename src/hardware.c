// BMS MASTER
//
// @author Louis-Philippe Asselin
// @date 26-7-18
//
// Initialisation and management of hardware 
//
#include "hardware.h"
#include "memory.h"
#include "defines.h"
#include "globals.h"
#include "ISR.h"
#include "SPI.h"
#include "SCI.h"

  ///////////////////////
 /// Local Variables ///
///////////////////////

#define EPWM_TIMER_TBPRD  5000U // TBRD = TPWM/2 * TBCLK

////////////////
/// Helpers ///
//////////////

static void pinCfg(uint16_t pin_number, uint32_t pin_map, GPIO_Direction direction){
    GPIO_setMasterCore(pin_number, GPIO_CORE_CPU1);
    GPIO_setPinConfig(pin_map);
    GPIO_setDirectionMode(pin_number, direction);
    GPIO_setPadConfig(pin_number, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(pin_number, GPIO_QUAL_ASYNC);
}

static inline void initGPIO()
{
    // see defines.h for all pins info

    pinCfg(GPIO_MCU_PUMP_CONTROL, PIN_MAP_MCU_PUMP_CONTROL, GPIO_DIR_MODE_OUT);
    //pinCfg(GPIO_PWM_MCU_FAN_CONTROL, PIN_MAP_PWM_MCU_FAN_CONTROL, GPIO_DIR_MODE_OUT);

    pinCfg(GPIO_MCU_DEBUG_LED_0, PIN_MAP_MCU_DEBUG_LED_0, GPIO_DIR_MODE_OUT);
    pinCfg(GPIO_MCU_DEBUG_LED_1, PIN_MAP_MCU_DEBUG_LED_1, GPIO_DIR_MODE_OUT);
    pinCfg(GPIO_MCU_DEBUG_LED_2, PIN_MAP_MCU_DEBUG_LED_2, GPIO_DIR_MODE_OUT);
    pinCfg(GPIO_MCU_XLKOUT, PIN_MAP_MCU_XLKOUT, GPIO_DIR_MODE_OUT);

    pinCfg(GPIO_MCU_CHARGER_DETECT, PIN_MAP_MCU_CHARGER_DETECT, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_OCD_PROBE, PIN_MAP_MCU_OCD_PROBE, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_TS_DISABLED, PIN_MAP_MCU_TS_DISABLED, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_FAN_NOT_RUNNING, PIN_MAP_MCU_FAN_NOT_RUNNING, GPIO_DIR_MODE_IN);

    pinCfg(GPIO_MCU_INTLCK_AMS, PIN_MAP_MCU_INTLCK_AMS, GPIO_DIR_MODE_OUT);

    pinCfg(GPIO_MCU_24V_INTLCK, PIN_MAP_MCU_24V_INTLCK, GPIO_DIR_MODE_IN);

    pinCfg(GPIO_MCU_INTLCK_AMS_IN, PIN_MAP_MCU_INTLCK_AMS_IN, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_INTLCK_HVD_OUT, PIN_MAP_MCU_INTLCK_HVD_OUT, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_INTLCK_HVD_IN, PIN_MAP_MCU_INTLCK_HVD_IN, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_INTLCK_OUT_2, PIN_MAP_MCU_INTLCK_OUT_2, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_INTLCK_IN_2, PIN_MAP_MCU_INTLCK_IN_2, GPIO_DIR_MODE_IN);

    pinCfg(GPIO_MCU_IMD_OK, PIN_MAP_MCU_IMD_OK, GPIO_DIR_MODE_IN);
    pinCfg(GPIO_MCU_DATA_OUT_HIGH, PIN_MAP_MCU_DATA_OUT_HIGH, GPIO_DIR_MODE_IN);

    // Write defaults
    GPIO_writePin(GPIO_MCU_DEBUG_LED_0, 1);
    GPIO_writePin(GPIO_MCU_DEBUG_LED_1, 1);
    GPIO_writePin(GPIO_MCU_DEBUG_LED_2, 1);
    GPIO_writePin(GPIO_MCU_PUMP_CONTROL, 0);
    //GPIO_writePin(GPIO_PWM_MCU_FAN_CONTROL, 0);
}

/**
 * Initialize PWM for the fans
 */
static inline void initPWM() {
    //Interrupt_register(INT_EPWM2, &epwm2ISR);
    GPIO_setPadConfig(GPIO_PWM_MCU_FAN_CONTROL, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(PIN_MAP_PWM_MCU_FAN_CONTROL);

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(PWM_MCU_FAN_BASE, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(PWM_MCU_FAN_BASE, 0U);
    EPWM_setTimeBaseCounter(PWM_MCU_FAN_BASE, 0U);

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(PWM_MCU_FAN_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(PWM_MCU_FAN_BASE);
    EPWM_setClockPrescaler(PWM_MCU_FAN_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);


    EPWM_setCounterCompareValue(PWM_MCU_FAN_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                0);
    //
    // Set-up shadowing
    //

    EPWM_setCounterCompareShadowLoadMode(PWM_MCU_FAN_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //

    EPWM_setActionQualifierAction(PWM_MCU_FAN_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(PWM_MCU_FAN_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

static inline void initADCs() {
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);

    // 16 bit resolution requires differential mode
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);

    ADC_enableConverter(ADCA_BASE);
    ADC_enableConverter(ADCB_BASE);

    // Configure ADC to trigger conversions on EPWM capture compare

    // EPWM ADC - begin
    ADC_setupSOC(ADCA_BASE, MCU_CURRENT_PROBE_ADC_SOC, ADC_TRIGGER_EPWM1_SOCA, MCU_CURRENT_PROBE_ADC_CH,15);
    ADC_setupSOC(ADCA_BASE, MCU_REFERENCE_CURRENT_PROBE_ADC_SOC, ADC_TRIGGER_EPWM1_SOCA, MCU_REFERENCE_CURRENT_PROBE_ADC_CH, 15);
    ADC_setupSOC(ADCA_BASE, MCU_DATA_OUT_HIGH_FILTERED_SOC, ADC_TRIGGER_EPWM1_SOCA, MCU_DATA_OUT_HIGH_FILTERED_CH, 15);
    ADC_setupSOC(ADCA_BASE, MCU_TEMP_MEASUREMENT_ADC_SOC, ADC_TRIGGER_EPWM1_SOCA, MCU_TEMP_MEASUREMENT_ADC_CH, 15);
    ADC_setupSOC(ADCA_BASE, MCU_HV_MEASUREMENT_ADC_SOC, ADC_TRIGGER_EPWM1_SOCA, MCU_HV_MEASUREMENT_ADC_CH, 15);

    ADC_setupSOC(ADCB_BASE, MCU_1V2_SOC, ADC_TRIGGER_EPWM1_SOCB, MCU_1V2_CH, 15);
    ADC_setupSOC(ADCB_BASE, MCU_3V3_SOC, ADC_TRIGGER_EPWM1_SOCB, MCU_3V3_CH, 15);
    ADC_setupSOC(ADCB_BASE, MCU_5V_SOC, ADC_TRIGGER_EPWM1_SOCB, MCU_5V_CH, 15);

    // Disable SOCx
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_B);

    // Configure the SOC to occur on the first up-count event

    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_B, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_B, 4); // 20khz/4

    // used but synced with interrupt??
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 1250);

    // 1/((2e-8)*CURRENT_PROBE_SAMPLING_FREQ) = period
    EPWM_setTimeBasePeriod(EPWM1_BASE, 2500); // 20kHz

    // triggers ADC
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_B);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
    // EPWM ADC - end

    // The interrupt source for ADC should be set to the highest sampled SOC number
    // because the ADC samples SOC's in order. (Soc number != PIN NUMBER)
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER4);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    Interrupt_register(INT_ADCA1, &adcA1ISR);
    Interrupt_enable(INT_ADCA1);

    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);

    Interrupt_register(INT_ADCB1, &adcB1ISR);
    Interrupt_enable(INT_ADCB1);
}

static void configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    //
    // Initialize timer period:
    //
    temp = (uint32_t)(freq / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);
}

static inline void init1sTimer(void){
    //Initialization of CPU TIMER
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    Interrupt_register(INT_TIMER0, &ISR_1s_timer);
    configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000000);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);

    CPUTimer_startTimer(CPUTIMER0_BASE);
}

static inline void init1msTimer(void){
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);

    Interrupt_register(INT_TIMER1, &ISR_1ms_timer);
    configCPUTimer(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, 1000);
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
    Interrupt_enable(INT_TIMER1);

    CPUTimer_startTimer(CPUTIMER1_BASE);
}

static inline void init30sTimer(void){
    //Initialization of CPU TIMER
    CPUTimer_setPeriod(CPUTIMER2_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    Interrupt_register(INT_TIMER2, &ISR_30s_timer);
    configCPUTimer(CPUTIMER2_BASE, DEVICE_SYSCLK_FREQ, 30000000);
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
    Interrupt_enable(INT_TIMER2);

    CPUTimer_startTimer(CPUTIMER2_BASE);
}

static inline void watchdog_diagnostic()
{
    // Check if reset from watchdog was performed
    if (SysCtl_getWatchdogResetStatus() == 1)
        {
#if (DEBUG == 1)
            //print("\r\nWatchdog reset device.\n");
            //DEVICE_DELAY_US(100000);
            //ESTOP0;
#elif (DEBUG == 0)
            // XXX what to do in case of watchdog reset?
#endif
        }
    SysCtl_clearWatchdogResetStatus();
}

static inline void initWatchdog()
{
    // Main loop Hardware watchdog
    watchdog_diagnostic();

    SysCtl_disableWatchdogStandbyWakeup();
    SysCtl_disableWatchdogInHalt();

#if(WATCHDOG_MODE_INTERRUPT == 1)
    Interrupt_register(INT_WAKE, &ISR_watchdog);
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_INTERRUPT);
    Interrupt_enable(INT_WAKE);
#elif(WATCHDOG_MODE_INTERRUPT == 0)
    SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);
#endif

    // Watchdog frequency: (10MHz / 512) / 64 * (1/256) = 1.1920929Hz = 0.8388608 seconds
    // 256: overflow of watchdog's 8 bit counter
    SysCtl_setWatchdogPrescaler(SYSCTL_WD_PRESCALE_64);
    SysCtl_serviceWatchdog();
    SysCtl_enableWatchdog();
}

static inline void checkIMDAndAMSIn(bool imd_ok, bool ams_in) {
    if(g_Time < 5) {
        return;
    }
    if (imd_ok == 1 && ams_in == 1) {
        // OK
    }
    else if (imd_ok == 0 && ams_in == 0) {// if IMD is closed and AMS confirms it
        // IMD says it is open
        set_error_bit(ERROR_INTERLOCK_IMD_OPEN);
    }
    else if (imd_ok == 0 && ams_in == 1) {// if IMD says it is open but it is closed according to AMS_IN
        set_error_bit(ERROR_INTERLOCK_IMD_NOT_OPEN);
    }
    else if (imd_ok == 1 && ams_in == 0) {// if IMD says it is closed but it is open according to AMS_IN
        set_error_bit(ERROR_INTERLOCK_IMD_NOT_CLOSED);
    }
}

static inline void checkAMSAndHVDOut(_Bool ams_output_closed, _Bool hvd_out) {
    // HVD_IN is the output of the HVD module. HVD_OUT is the input.
    if (ams_output_closed == 1 && hvd_out == 1) {
        // OK
    }
    else if (ams_output_closed == 0 && hvd_out == 0) {// check if AMS is open and HVD_OUT confirms
        set_error_bit(ERROR_INTERLOCK_AMS_OPEN);
    }
    else if (ams_output_closed == 0 && hvd_out == 1) {// check HVD_OUT confirms AMS is open if it is asked to be
        set_error_bit(ERROR_INTERLOCK_AMS_NOT_OPEN);
    }
    else if (ams_output_closed == 1 && hvd_out == 0) {// check HVD_OUT confirms AMS is closed if it is asked to be
        // This is not an error, it could be latching circuit
        //set_error_bit(ERROR_INTERLOCK_AMS_NOT_CLOSED);
    }
}


static inline void checkHVD(_Bool hvd_out, _Bool hvd_in) {
    if (hvd_out == 1 && hvd_in == 1) {
        // OK
        // Autoclear non critical errors
        clear_error_bit(ERROR_INTERLOCK_HVD);
    }
    else if (hvd_out == 0 && hvd_in == 0) {
        // OK
    }
    else if (hvd_out == 1 && hvd_in == 0) {// check if HVD module does not work properly
        set_error_bit(ERROR_INTERLOCK_HVD);
    }
    else if (hvd_out == 0 && hvd_in == 1) {// confirm HVD works only when it should
        set_error_bit(ERROR_INTERLOCK_HVD_IN_SHOULD_NOT_WORK);
    }
}

static inline void checkInterlockOtherVehiculeComponentsLine1(_Bool hvd_in, _Bool interlock_out_2) {
    // check signal going to other components of the vehicule
    // HVD_IN same as INTERLOCK_OUT_1
    // interlock_out -> loop_in_vehicule -> interlock_in
    if (hvd_in == 1 && interlock_out_2 == 1) {
        // OK
    }
    else if (hvd_in == 0 && interlock_out_2 == 0) {
        // OK
    }
    else if (hvd_in == 1 && interlock_out_2 == 0) {
        set_error_bit(ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_1);
    }
    else if (hvd_in == 0 && interlock_out_2 == 1) {
        set_error_bit(ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_1_SHOULD_NOT_WORK);
    }
}

static inline void checkInterlockOtherVehiculeComponentsLine2(_Bool interlock_out_2, _Bool interlock_in_2) {
    // check signal going to other components of the vehicule
    // interlock_out -> loop_in_vehicule -> interlock_in
    // INTERLOCK_IN_1 same as INTERLOCK_OUT_2
    if (interlock_out_2 == 1 && interlock_in_2 == 1) {
        // OK
    }
    else if (interlock_out_2 == 0 && interlock_in_2 == 0) {
        // OK
    }
    else if (interlock_out_2 == 1 && interlock_in_2 == 0) {
        set_error_bit(ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_2);
    }
    else if (interlock_out_2 == 0 && interlock_in_2 == 1) {
        set_error_bit(ERROR_INTERLOCK_OTHER_VEHICULE_COMPONENT_LINE_2_SHOULD_NOT_WORK);
    }
}

///////////////////////////
/// Low Level functions ///
///////////////////////////

bool charger_connected(){return GPIO_readPin(GPIO_MCU_CHARGER_DETECT);}
void startPump(){GPIO_writePin(GPIO_MCU_PUMP_CONTROL, 1);}
void stopPump(){GPIO_writePin(GPIO_MCU_PUMP_CONTROL, 0);}
void manageFan(){
    //GPIO_writePin(GPIO_PWM_MCU_FAN_CONTROL, 1);
    uint16_t new_cmp =(EPWM_TIMER_TBPRD/255) * (threading.fanStartDelay == 0? 0 : MAX(threading.fanStartDelay, 28));
    EPWM_setCounterCompareValue(PWM_MCU_FAN_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                new_cmp);
}
bool isFanRunningSoftware(){return threading.fanStartDelay != 0;}
void check_fan(){
#if DISABLE_FAN_OK==0
    if (is_fan_running_software() && !GPIO_readPin(GPIO_MCU_FAN_NOT_RUNNING)){
        set_error_bit(ERROR_FAN_SHOULD_BE_RUNNING);
    }
#else
#warning "Fan check is disabled"
#endif
}
void setLed0(){GPIO_writePin(GPIO_MCU_DEBUG_LED_0, 1);}
void setLed1(){GPIO_writePin(GPIO_MCU_DEBUG_LED_1, 1);}
void setLed2(){GPIO_writePin(GPIO_MCU_DEBUG_LED_2, 1);}
void toggleLed0(){GPIO_togglePin(GPIO_MCU_DEBUG_LED_0);}
void toggleLed1(){GPIO_togglePin(GPIO_MCU_DEBUG_LED_1);}
void toggleLed2(){GPIO_togglePin(GPIO_MCU_DEBUG_LED_2);}
void clearLed0(){GPIO_writePin(GPIO_MCU_DEBUG_LED_0, 0);}
void clearLed1(){GPIO_writePin(GPIO_MCU_DEBUG_LED_1, 0);}
void clearLed2(){GPIO_writePin(GPIO_MCU_DEBUG_LED_2, 0);}
void closeInterlock() {
    clear_error_bit(ERROR_INTERLOCK_AMS_OPEN);
    if (g_error_status == (uint64_t) 0 ){
        GPIO_writePin(GPIO_MCU_INTLCK_AMS, 1);
    }
}
void openInterlock(){GPIO_writePin(GPIO_MCU_INTLCK_AMS, 0);}
bool interlockDetect(){return GPIO_readPin(GPIO_MCU_INTLCK_IN_2);}

  /////////////////////
 /// Implementaton ///
/////////////////////


void initialize()
{
    // Configure PLL, disable WD, enable peripheral clocks.
    Device_init();

    // Disable pin locks and enable internal pullups.
    Device_initGPIO();

    Interrupt_initModule();
    Interrupt_initVectorTable();

    init_SPI();
    init_SCI();

    initADCs();

    initWatchdog();

    initGPIO();
    initPWM();

    //startPump();

    init1sTimer();
    init1msTimer();
    init30sTimer();

    initializeGlobals();
    reset_all_g_EventCount();

#ifndef USING_LAUNCHPAD // This will stall if using launchpad hence no EEPROM will answer to SPI..
    protectEepromMemory(); // Locks writing to EEPROM so you don't fuck up.
#endif

    // Uncomment to set defaults to eeprom (or use CAN fct)
    // Might be useful to trigger param write to EEPROM
    /*EEPROM_unprotect_memory();
    save_checkpoint_to_EEPROM();
    save_error_status();
    EEPROM_protect_memory();*/

#ifndef USING_LAUNCHPAD // This will stall if using launchpad hence no EEPROM will answer to SPI...
    loadCheckpointFromEeprom();
#endif

    init_CAN();

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    slave_initialize();
#if (NO_SLAVE_TEST == 0)
    slave_test();
#else
#warning "No slave tests!"
#endif

    g_temperatureSensorIdx = 0;
    g_errorTimerMs = 0;

    DEVICE_DELAY_US(1000);

}


void checkInterlockCircuit(){
    // read all necessary values for diagnosis (inputs and outputs)
    bool interlock_24V_ok = GPIO_readPin(GPIO_MCU_24V_INTLCK);
    bool imd_ok = GPIO_readPin(GPIO_MCU_IMD_OK);
    bool ams_in = GPIO_readPin(GPIO_MCU_INTLCK_AMS_IN);
    bool ams_output_closed = GPIO_readPin(GPIO_MCU_INTLCK_AMS);
    bool hvd_out = GPIO_readPin(GPIO_MCU_INTLCK_HVD_OUT);
    bool hvd_in = GPIO_readPin(GPIO_MCU_INTLCK_HVD_IN);
    bool interlock_out_2 = GPIO_readPin(GPIO_MCU_INTLCK_OUT_2);
    bool interlock_in_2 = GPIO_readPin(GPIO_MCU_INTLCK_IN_2);

    // When a check fails, all others after him should also fail. If that is not the case, error flags are set.
    // Only the flag related to the failing component in interlock line is set as error.
    // Check 24V interlock
    if (!interlock_24V_ok) {
        set_error_bit(ERROR_INTERLOCK_24V);
    }
    checkIMDAndAMSIn(imd_ok, ams_in);
    checkAMSAndHVDOut(ams_output_closed, hvd_out);
    checkHVD(hvd_out, hvd_in);
    checkInterlockOtherVehiculeComponentsLine1(hvd_in, interlock_out_2);
    checkInterlockOtherVehiculeComponentsLine2(interlock_out_2, interlock_in_2);

    g_masterBMS.interlock_no_error = check_interlock_no_error();
}


