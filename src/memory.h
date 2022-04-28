/*
 * memory.h
 *
 *  Created on: 21 mai 2018
 *      Author: Nico
 */

#ifndef SRC_MEMORY_H_
#define SRC_MEMORY_H_

#include <device.h>
#include "defines.h"

/**
 * Loads SOC and parameters from EEPROM on boot
 */
void loadCheckpointFromEeprom(void);
/**
 * Saves SOC and parameters in EEPROM (safe state checkpoint)
 */
void saveCheckpointToEeprom(void);
void saveErrorStatus(void);

void protectEepromMemory(void);
void unprotectEepromMemory(void);


#endif /* SRC_MEMORY_H_ */
