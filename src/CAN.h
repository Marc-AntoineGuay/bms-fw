/*
 * CAN.h
 *
 *  Created on: Mar 22, 2018
 *      Author: Nicolas Juteau
 */

#ifndef SRC_CAN_H_
#define SRC_CAN_H_

#include <device.h>
#include "defines.h"
#include "messaging.h"

void init_CAN();
void setupMessages_CAN();

// Why aren't those method available in the driverlib anyway? 0^o
uint16_t CAN_readMessage2(uint32_t base, uint32_t objID, uint32_t *fullMessageId, uint16_t *msgData);

void CAN_setupBitRate_Charger();
void CAN_setupBitRate_Normal();

#endif /* SRC_CAN_H_ */
