// BMS MASTER
//
// @author Guillaume Soulard
// @date 26-7-18
//
// Initialisation and management of hardware 
//

#ifndef SRC_HARDWARE_H_
#define SRC_HARDWARE_H_

#include <device.h>

/**
 * Initialize hardware components
 */
void initialize(void);

/**
 * Check interlock circuit and set appropriate errors
 */
void check_interlock_circuit(void);
///////////////////////////
/// Low Level functions ///
///////////////////////////

void openInterlock(void);
/**
 * Closes interlock only if error status is valid
 */
void closeInterlock(void);
bool interlockDetect(void);

bool charger_connected(void);

void startPump(void);
void stopPump(void);

void manageFan(void);
bool isFanRunningSoftware(void);
/**
 * Checks if fan should be running and sets error
 * Does not check if fan should not be running and is running.
 */
void checkFan(void);

void setLed0(void);
void setLed1(void);
void setLed2(void);

void toggleLed0(void);
void toggleLed1(void);
void toggleLed2(void);

void clearLed0(void);
void clearLed1(void);
void clearLed2(void);

inline float convert_ref_1V2(uint16_t adc_value){
    return 3.3* (float)adc_value / 4096;
}

inline float convert_ref_5V(uint16_t adc_value){
    return 2* 3.3* (float)adc_value / 4096;
}

inline float convert_ref_3V3(uint16_t adc_value){
    // IMPORTANT: the factor for g_ref3V3 is 2.2 because there is 50k pulldown in parallel with the 10k. This pulldown cannot be disabled.
    return 2.2* 3.3* (float)adc_value / 4096;
}


#endif /* SRC_HARDWARE_H_ */
