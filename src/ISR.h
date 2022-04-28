/*
 * ISR.h
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_ISR_H_
#define SRC_ISR_H_

#include <device.h>
#include "defines.h"
#include "actions.h"
#include "CAN.h"

__interrupt void ISR_1s_timer();
__interrupt void ISR_1ms_timer();
__interrupt void ISR_30s_timer();

#if (WATCHDOG_MODE_INTERRUPT == 1)
__interrupt void ISR_watchdog();
#endif

__interrupt void adcA1ISR(void);
__interrupt void adcB1ISR(void);

__interrupt void ISR_CAN(void);

#endif /* SRC_ISR_H_ */
