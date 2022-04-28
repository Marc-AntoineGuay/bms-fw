/*
 * error_management.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_ERROR_MANAGEMENT_H_
#define SRC_ERROR_MANAGEMENT_H_

#include "defines.h"
#include "hardware.h"
#include "messaging.h"
#include "memory.h"

void clear_all_errors(void);
void error_check_from_reset(void);
void reset_all_g_EventCount(void);
bool error_management(void);
bool check_interlock_no_error();

#endif /* SRC_ERROR_MANAGEMENT_H_ */
