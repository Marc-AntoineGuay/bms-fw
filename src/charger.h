/*
 * charger.h
 *
 *  Created on: 16 avr. 2018
 *      Author: Nicolas Juteau
 */

#ifndef SRC_CHARGER_H_
#define SRC_CHARGER_H_

#include <device.h>
#include "defines.h"
#include "messaging.h"
#include "CAN.h"
#include "slave/LTC6804-1.h"
#include "error_management.h"

void charger_compute_maximums(struct Charger *);
void manage_charger_states();

#endif /* SRC_CHARGER_H_ */
