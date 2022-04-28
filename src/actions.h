/*
 * actions.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_ACTIONS_H_
#define SRC_ACTIONS_H_

#include <device.h>
#include "defines.h"
#include "slave/slave.h"

inline bool ready_read_CV_and_temperature(){
    return g_slaveMeasurementCountMs > (MEAS_MS);
}
inline bool ready_save_data_eeprom(){
    return g_saveDataEepromSeconds > (SECONDS_BETWEEN_DATA_SAVE);
}

/**
 * Find actual BMS state
 */
void manage_status(void);

void perform_actions(void);

#endif /* SRC_ACTIONS_H_ */
