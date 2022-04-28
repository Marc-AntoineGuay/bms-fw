/*
 * SCI.h
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_SCI_H_
#define SRC_SCI_H_

#include <device.h>
#include "defines.h"

void init_SCI();
void print(char * str);
void print_hex(uint16_t data);

#endif /* SRC_SCI_H_ */
