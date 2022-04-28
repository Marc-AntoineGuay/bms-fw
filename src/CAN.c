/*
 * CAN.c
 *
 *  Created on: Mar 22, 2018
 *      Author: Nicolas Juteau
 */

#include "defines.h"
#include "messaging.h"
#include "CAN.h"
#include "ISR.h"

void init_CAN()
{
    // GPIO setup
    GPIO_setMasterCore(GPIO_MCU_CAN_RX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_CAN_RX);
    GPIO_setPadConfig(GPIO_MCU_CAN_RX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_CAN_RX, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_MCU_CAN_TX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_CAN_TX);
    GPIO_setPadConfig(GPIO_MCU_CAN_TX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_CAN_TX, GPIO_QUAL_ASYNC);

    // CAN module setup
    CAN_initModule(MCU_CAN_BASE);
    CAN_setBitRate(MCU_CAN_BASE, DEVICE_SYSCLK_FREQ, MCU_CAN_FREQ, 16);
    // XXX: check for a difference between IE0 and IE1, if any
    CAN_enableInterrupt(MCU_CAN_BASE, CAN_INT_IE0 | CAN_INT_IE1 | CAN_INT_ERROR | CAN_INT_STATUS);

    // ISR setup
    Interrupt_register(MCU_CAN_INT, &ISR_CAN);
    Interrupt_enable(MCU_CAN_INT);
    CAN_enableGlobalInterrupt(MCU_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

#if (CAN_INTERNAL_DEBUG==1)
    // This will enable CAN core in test mode (w/ external loopback)
    // Frame acknowledgement error (from receiver) will be ignored
    // and the tx signal is fed to the external TX pin, thus allowing can transceiver to be tested
    CAN_enableTestMode(MCU_CAN_BASE, CAN_TEST_EXL);
#endif

    // Setup CAN message objects
    // IMPORTANT: The receive/transmit priority for the message objects is attached to the message number, not to the CAN identifier.
    //            Message object 1 has the highest priority, while message object 32 has the lowest priority.

    // Assign messages to CAN mailboxes
    setupMessages_CAN();

    // Start CAN module
    CAN_startModule(MCU_CAN_BASE);

}

void setupMessages_CAN() {
    for (int i = 1; g_Messages[i].id != 0; i++) {
        //g_Messages[i].id = i;

        CAN_MsgObjType msgObjType = g_Messages[i].tx ? CAN_MSG_OBJ_TYPE_TX : CAN_MSG_OBJ_TYPE_RX;
        uint32_t msgMask = 0;
        if (g_Messages[i].tx == 0) {
            // RX message
            msgMask = g_Messages[i].raw_mask == 0 ? MSG_MASK_RX : MSG_MASK_RX_RAW;
        }

        uint32_t msgFlags = 0;
        if (g_Messages[i].tx == 0) {
            msgFlags |= CAN_MSG_OBJ_RX_INT_ENABLE | CAN_MSG_OBJ_USE_EXT_FILTER | CAN_MSG_OBJ_USE_ID_FILTER;
        }
        else {
            msgFlags |= CAN_MSG_OBJ_TX_INT_ENABLE;
        }

        uint32_t msgId = msg_getMsgId(g_Messages[i]);
        CAN_setupMessageObject(MCU_CAN_BASE, i, msgId, CAN_MSG_FRAME_EXT, msgObjType, msgMask, msgFlags, MAX_FRAME_LENGTH);
    }

}

uint16_t
CAN_readMessage2(uint32_t base, uint32_t objID, uint32_t *fullMessageId, uint16_t *msgData)
{
    int16_t msgLen = 0U;
    uint16_t msgCtrl = 0U;

    //
    // Check the arguments.
    //
    ASSERT(CAN_isBaseValid(base));
    ASSERT((objID <= 32U) && (objID != 0U));

    //
    // Set the Message Data A, Data B, and control values to be read
    // on request for data from the message object.
    //
    // Transfer the message object to the message object IF register.
    //
    HWREG_BP(base + CAN_O_IF2CMD) = (CAN_IF2CMD_DATA_A | CAN_IF2CMD_DATA_B |
                                     CAN_IF2CMD_CONTROL | CAN_IF2CMD_ARB |
                                     (objID & CAN_IF2CMD_MSG_NUM_M));

    //
    // Wait for busy bit to clear
    //
    while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
    {
    }

    //
    // Read out the IF control Register.
    //
    msgCtrl = HWREGH(base + CAN_O_IF2MCTL);

    //
    // See if there is new data available.
    //
    if((msgCtrl & CAN_IF2MCTL_NEWDAT) == CAN_IF2MCTL_NEWDAT)
    {
        // Read out arbitration control register to fetch the full message id that triggered the interrupt
        *fullMessageId = HWREG_BP(base + CAN_O_IF2ARB) & CAN_IF1ARB_ID_M;

        //
        // Read out the data from the CAN registers.
        //
        CAN_readDataReg(msgData, (int16_t *)(base + CAN_O_IF2DATA),
                        (msgCtrl & CAN_IF2MCTL_DLC_M));

        msgLen = (uint16_t)(msgCtrl & CAN_IF2MCTL_DLC_M);

        //
        // Now clear out the new data flag
        //
        // Transfer the message object to the message object specified by
        // objID.
        //
        HWREG_BP(base + CAN_O_IF2CMD) = CAN_IF2CMD_TXRQST |
                                             (objID & CAN_IF2CMD_MSG_NUM_M);

        //
        // Wait for busy bit to clear
        //
        while((HWREGH(base + CAN_O_IF2CMD) & CAN_IF2CMD_BUSY) == CAN_IF2CMD_BUSY)
        {
        }
    }
    else
    {
        msgLen = 0U;
    }

    return(msgLen);
}

void CAN_setupBitRate_Charger() {
    CAN_setBitRate(MCU_CAN_BASE, DEVICE_SYSCLK_FREQ, MCU_CHARGER_CAN_FREQ, 16);
}

void CAN_setupBitRate_Normal() {
    CAN_setBitRate(MCU_CAN_BASE, DEVICE_SYSCLK_FREQ, MCU_CAN_FREQ, 16);
}
