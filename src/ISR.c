/*
 * ISR.c
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "ISR.h"
#include "globals.h"

#if (WATCHDOG_MODE_INTERRUPT == 1)
uint16_t wakeCount;
__interrupt void ISR_watchdog()
{
    // This interrupt should only be called in DEBUG, instead of resetting the device.
    //ESTOP0;
    wakeCount++;

    // Acknowledge this interrupt
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    GPIO_togglePin(DEVICE_GPIO_PIN_LED1);


}
#endif


uint16_t balancingCounter = 0;
__interrupt void ISR_1s_timer(void)
{
    toggleLed0();

    g_saveDataEepromSeconds++;
    g_Time++;

    if(++errorCVTimer.oor > CV_DELAY_ERROR) { set_error_bit(ERROR_CV_OUT_OF_RANGE); }
    if(++errorCVTimer.ovuv > CV_DELAY_ERROR) { set_error_bit(ERROR_CV_UVOV); }


    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void ISR_1ms_timer(void)
{
    // toggleLed0();
    static uint32_t can_tx_timer = 0;
    if ((++can_tx_timer) % 1000 == 0) {
        // Schedule periodic message for sending
        msg_schedule_tx(MSG_ID_RESP_POWER_LIMIT);
        msg_schedule_tx(MSG_ID_RESP_MASTERBMS_STATE);
        msg_schedule_tx(MSG_ID_RESP_ERROR_STATUS);
        msg_schedule_tx(MSG_ID_RESP_TSMS);
    } else if (can_tx_timer % 1000 == 333) {
        msg_schedule_tx(MSG_ID_RESP_CVS);
        msg_schedule_tx(MSG_ID_RESP_CVS_FLAGS);
        msg_schedule_tx(MSG_ID_RESP_CVS_BALANCE);
    } else if (can_tx_timer % 1000 == 666) {
        msg_schedule_tx(MSG_ID_RESP_TEMPS);
        msg_schedule_tx(MSG_ID_RESP_TEMPS_FLAGS);
    }

    static uint64_t hz = 0;
    if((++hz) % 256 == 0) {
        if(threading.fanStartDelay > 0 && threading.fanStartDelay < 255) {
            ++threading.fanStartDelay;
        }


    }

    if((hz % 32) == 0) {
        // NOTE Sending CAN charger message at 4 Hz
        // If charger not connected anymore, reset CAN baudrate to normal
        if (!g_masterBMS.charger_connected) {
            CAN_setupBitRate_Normal();
        }
        // NOTE CHANGED FROM check_interlock_no_error
        else if (check_interlock_no_error()) {
            // Transmit messages to charger if we are in charge safe conditions
            msg_schedule_tx(MSG_ID_CHARGER_TRANSMIT);
        }
    }

    g_slaveMeasurementCountMs++;
    g_errorTimerMs++;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//this is only used when charging the battery, it turns off balancing, measures the values and turns on balancing with the new config
__interrupt void ISR_30s_timer(void)
{
    if (g_BatteryPack.charging){

        bool balancingOn = false;

        for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
            if (g_BatteryPack.slave[idx_slave].cellDischargeON != 0) {
                balancingOn = true;
            }
        }

        if (balancingOn && balancingCounter >= 2) { // TODO RAISE VALUE
            for (uint16_t idx_slave = 0; idx_slave < TOTAL_SLAVES; idx_slave++) {
                g_BatteryPack.slave[idx_slave].cellDischargeON = 0;
                g_BatteryPack.slave[idx_slave].balancingJustFinish = 0;
                slave_update_cell_discharge_config(idx_slave, 0);
            }
            slave_write_current_config();
            balancingCounter = 0;
        }
        else
        {
            if (balancingCounter == 0) {
                manage_balancing_config();
                slave_write_current_config();
            }
            balancingCounter++;
        }
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

/**
 * @brief: Reads current probe ADC and adds data to battery pack coulomb counter
 *
 * Checks if current probe is out of range
 * Checks if current value is too high
 */
__interrupt void adcA1ISR(void)
{
    //toggleLed0();

    g_EventCount.current_probe_total++;
    g_RawAdcValues.current_probe = ADC_readResult(ADCARESULT_BASE, MCU_CURRENT_PROBE_ADC_SOC);
    g_RawAdcValues.current_probe_ref = ADC_readResult(ADCARESULT_BASE, MCU_REFERENCE_CURRENT_PROBE_ADC_SOC);
    g_RawAdcValues.imd_data_out = (int16_t)ADC_readResult(ADCARESULT_BASE, MCU_DATA_OUT_HIGH_FILTERED_SOC);
    g_RawAdcValues.master_temperature = (int16_t)ADC_readResult(ADCARESULT_BASE, MCU_TEMP_MEASUREMENT_ADC_SOC);
    g_RawAdcValues.hv_measurement = (int16_t)ADC_readResult(ADCARESULT_BASE, MCU_HV_MEASUREMENT_ADC_SOC);

    g_RawAdcValues.process_groupA_ready = true;

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void adcB1ISR(void)
{
    //toggleLed0();

    g_RawAdcValues.pwr_5v = (uint16_t) ADC_readResult(ADCBRESULT_BASE, MCU_5V_SOC);
    g_RawAdcValues.pwr_3v3 = (uint16_t) ADC_readResult(ADCBRESULT_BASE, MCU_3V3_SOC);
    g_RawAdcValues.pwr_1v2 = (uint16_t)ADC_readResult(ADCBRESULT_BASE, MCU_1V2_SOC);
    g_RawAdcValues.process_groupB_ready = true;

    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

bool CAN_errorFlag;

__interrupt void ISR_CAN(void)
{
    uint32_t status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    status = CAN_getInterruptCause(MCU_CAN_BASE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        status = CAN_getStatus(MCU_CAN_BASE);

        //
        // Check to see if an error occurred.
        //
        if(((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 7) &&
           ((status  & ~(CAN_STATUS_TXOK | CAN_STATUS_RXOK)) != 0))
        {
            CAN_errorFlag=1;
        }
    }
    else
    {
        //
        // At this point, status contains the mailbox ID (message id object) that triggered the interrupt
        //

        if (g_Messages[status].tx) {
            g_Messages[status].tx_isr_waiting = true;
        }
        else {
            uint16_t rxMsgData[MAX_FRAME_LENGTH];
            uint32_t fullMsgId;

            uint16_t msgLen = CAN_readMessage2(MCU_CAN_BASE, status, &fullMsgId, rxMsgData);

            msg_processRx(status, fullMsgId, msgLen, rxMsgData);
        }

        CAN_clearInterruptStatus(MCU_CAN_BASE, status);
    }

    //
    // Clear the global interrupt flag for the CAN interrupt line
    //
    CAN_clearGlobalInterruptStatus(MCU_CAN_BASE, CAN_GLOBAL_INT_CANINT0);

    //
    // Acknowledge this interrupt located in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}
