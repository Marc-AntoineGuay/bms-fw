/*
 * messaging.c
 *
 *  Created on: 9 avr. 2018
 *      Author: Nicolas Juteau
 */

#include "messaging.h"

#define FRAME_ROUND(a) ((a) % (8) == 0 ? (a/8) : ((a/8) + 1))

#define MSG_CVS_FB_COUNT    FRAME_ROUND(TOTAL_SLAVES*CELL_VOLTAGES_PER_SLAVE) // 3
#define MSG_TEMP_FB_COUNT   FRAME_ROUND(TOTAL_SLAVES*TEMPERATURE_SENSORS_PER_SLAVE) // 4

#define MSG_FLAGS_FB_COUNT          FRAME_ROUND(2*TOTAL_SLAVES) // 2bytes required per slave to store cell voltage or temperature or balance flags
#define MSG_FLAGS_PAYLOAD_LENGTH    2*TOTAL_SLAVES  // 2 bytes required per slave and multiplied by 2 since we are sending bytes

Frame g_Message_Resp_PowerLimit_Fb[1];
Frame g_Message_Resp_Cvs_Fb[MSG_CVS_FB_COUNT];
Frame g_Message_Resp_Temps_Fb[MSG_TEMP_FB_COUNT];
Frame g_Message_Req_PeriodicValues_Fb[1];
Frame g_Message_Req_ResetErrorState_Fb[1];
Frame g_Message_Resp_ErrorStatus_Fb[1];
Frame g_Message_ChargerTransmit_Fb[1];
Frame g_Message_ChargerReceive_Fb[1];
Frame g_Message_Req_ManagePump_Fb[1];
Frame g_Message_Resp_MasterBMSState_Fb[2];
Frame g_Message_Resp_Cvs_Flags_Fb[MSG_FLAGS_FB_COUNT];
Frame g_Message_Resp_Temps_Flags_Fb[MSG_FLAGS_FB_COUNT];
Frame g_Message_Resp_Cvs_Balance_Fb[MSG_FLAGS_FB_COUNT];
Frame g_Message_Resp_TSMS[MSG_FLAGS_FB_COUNT];
Frame g_Message_Resp_CvTemp_Fb[1]; //COMMENTHERE

bool g_Message_SendPeriodicCvs = true;
bool g_Message_SendPeriodicTemps = true;
bool g_Message_SendPeriodicMasterState = true;
bool g_Message_SendPeriodicCvsFlags = true;
bool g_Message_SendPeriodicTempsFlags = true;
bool g_Message_SendPeriodicCvsBalance = true;



/*
 * MANDATORY TO DEFINE:
 *  - id
 *  - source
 *  - frame count
 *  - frames pointer !!!
 */
// IMPORTANT: MESSAGES MUST BE IN ORDER OF MESSAGE_ID (0, 1, 2, ...) no skipping
// It will be overwritten to follow this convention anyway.
#define G_MESSAGES_ARRAY_LENGTH 17
Message g_Messages[G_MESSAGES_ARRAY_LENGTH] = {
                        { .id = 0 },
                        { .id = MSG_ID_RESP_POWER_LIMIT, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 0, .request = 0, .tx = 1, .frame_count = 1, .frames = g_Message_Resp_PowerLimit_Fb, .func_ptr = msg_sendPowerLimit },
                        { .id = MSG_ID_RESP_CVS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_CVS_FB_COUNT, .frames = g_Message_Resp_Cvs_Fb, .func_ptr = msg_sendCvs },
                        { .id = MSG_ID_RESP_TEMPS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_TEMP_FB_COUNT, .frames = g_Message_Resp_Temps_Fb, .func_ptr = msg_sendTemps },
                        { .id = MSG_ID_REQ_PERIODIC_VALUES, .source = MSG_SOURCE_REQ_ECU_DASH_TELEMETRY_MASTER_CAN_NODE, .telemetry = 0, .request = 1, .frame_count = 1, .frames = g_Message_Req_PeriodicValues_Fb, .func_ptr = msg_ReqPeriodicValues },
                        { .id = MSG_ID_REQ_RESET_ERROR_STATE, .source = MSG_SOURCE_REQ_ECU_DASH_TELEMETRY_MASTER_CAN_NODE, .telemetry = 0, .request = 1, .frame_count = 1, .frames = g_Message_Req_ResetErrorState_Fb, .func_ptr = msg_ReqResetErrorState },
                        { .id = MSG_ID_CHARGER_TRANSMIT, .source = MSG_SOURCE_CHARGER, .tx = 1, .frame_count = 1, .frames = g_Message_ChargerTransmit_Fb, .func_ptr = msg_chargerTransmit, .raw_mask = MSG_RAW_MASK_CHARGER_TRANSMIT },
                        { .id = MSG_ID_RESP_ERROR_STATUS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = 1, .frames = g_Message_Resp_ErrorStatus_Fb, .func_ptr = msg_sendErrorStatus },
                        { .id = MSG_ID_REQ_MANAGE_PUMP, .source = MSG_SOURCE_REQ_BMS_ECU, .request = 1, .frame_count = 1, .frames = g_Message_Req_ManagePump_Fb, .func_ptr = msg_ReqManagePump },
                        { .id = MSG_ID_RESP_MASTERBMS_STATE, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = 2, .frames = g_Message_Resp_MasterBMSState_Fb, .func_ptr = msg_sendMasterBMSState },
                        { .id = MSG_ID_RESP_CVS_FLAGS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_FLAGS_FB_COUNT, .frames = g_Message_Resp_Cvs_Flags_Fb, .func_ptr = msg_sendCvsFlags },
                        { .id = MSG_ID_RESP_TEMPS_FLAGS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_FLAGS_FB_COUNT, .frames = g_Message_Resp_Temps_Flags_Fb, .func_ptr = msg_sendTempsFlags },
                        { .id = MSG_ID_RESP_CVS_BALANCE, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_FLAGS_FB_COUNT, .frames = g_Message_Resp_Cvs_Balance_Fb, .func_ptr = msg_sendCvsBalance }, //COMMENTHERE
                        { .id = MSG_ID_RESP_TSMS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 1, .frame_count = MSG_FLAGS_FB_COUNT, .frames = g_Message_Resp_TSMS, .func_ptr = msg_sendTSMS },
                        { .id = MSG_ID_RESP_CVTEMP, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 0, .request = 0, .tx = 1, .frame_count = 1, .frames = g_Message_Resp_CvTemp_Fb, .func_ptr = msg_sendCvTemp },
                        /* MSG MUST BE LAST */  { .id = MSG_ID_CHARGER_RECEIVE, .source = MSG_SOURCE_CHARGER, .frame_count = 1, .frames = g_Message_ChargerReceive_Fb, .func_ptr = msg_chargerReceive, .raw_mask = MSG_RAW_MASK_CHARGER_RECEIVE },
                        //{ .id = MSG_ID_RESP_CVS, .source = MSG_SOURCE_MASTER_BMS, .telemetry = 1, .request = 0, .tx = 0, .rx_func_ptr = msg_TestCallback}, // For RX test
                        { .id = 0} // Mark end of messages array
};
#define G_MESSAGES_LENGTH 15

void msg_sendCvsFlags(uint16_t msgId) {
    if (!g_Message_SendPeriodicCvsFlags) return;

    uint16_t payload[MSG_FLAGS_PAYLOAD_LENGTH];

    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){
        splitUint16InUint8(g_BatteryPack.slave[idx_slave].voltageFlags, &payload[2*idx_slave], &payload[(2*idx_slave)+1]);
    }

    msg_send(MSG_ID_RESP_CVS_FLAGS, MSG_FLAGS_PAYLOAD_LENGTH, payload);
}

void msg_sendTempsFlags(uint16_t msgId) {
    if (!g_Message_SendPeriodicTempsFlags) return;

    uint16_t payload[MSG_FLAGS_PAYLOAD_LENGTH];

    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){
        splitUint16InUint8(g_BatteryPack.slave[idx_slave].temperatureFlags, &payload[2*idx_slave], &payload[(2*idx_slave)+1]);
    }

    msg_send(MSG_ID_RESP_TEMPS_FLAGS, MSG_FLAGS_PAYLOAD_LENGTH, payload);
}

void msg_sendCvsBalance(uint16_t msgId) {
    if (!g_Message_SendPeriodicCvsBalance) return;

    uint16_t payload[MSG_FLAGS_PAYLOAD_LENGTH];

    for(uint16_t idx_slave=0; idx_slave < TOTAL_SLAVES; idx_slave++){
        splitUint16InUint8(g_BatteryPack.slave[idx_slave].cellDischargeON, &payload[2*idx_slave], &payload[(2*idx_slave)+1]);
    }

    msg_send(MSG_ID_RESP_CVS_BALANCE, MSG_FLAGS_PAYLOAD_LENGTH, payload);
}

void msg_sendMasterBMSState(uint16_t msgId) {
    if (!g_Message_SendPeriodicMasterState) return;

    int16_t tt_test = convertThermistanceToCelsius(g_masterBMS.temperature);

    uint16_t payload[15];

    uint16_t pwr_1V2 = (uint16_t)(convert_ref_1V2(g_masterBMS.pwr_1V2)*50);
    uint16_t pwr_3V3 = (uint16_t)(convert_ref_3V3(g_masterBMS.pwr_3V3)*50);
    uint16_t pwr_5V = (uint16_t)(convert_ref_5V(g_masterBMS.pwr_5V)*50);

    payload[0] = pwr_1V2;
    payload[1] = pwr_3V3;
    payload[2] = pwr_5V;
    payload[3] = convertMasterThermistanceToCelcius(g_masterBMS.temperature);

    splitUint16InUint8(g_masterBMS.current_probe_ref, &payload[4], &payload[5]);
    splitUint16InUint8(g_masterBMS.imd_data_out, &payload[6], &payload[7]);
    splitUint16InUint8(g_masterBMS.hv_measurement, &payload[8], &payload[9]);
    splitUint16InUint8(g_pecFailCount, &payload[10], &payload[11]);

    payload[12] = g_masterBMS.interlock_no_error;
    payload[13] = g_masterBMS.charger_connected;
    payload[14] = g_BatteryPack.charging;

    msg_send(MSG_ID_RESP_MASTERBMS_STATE, 15, payload);
}

void msg_ReqManagePump(uint16_t msgId) {
    /*bool pumpStatus = g_Messages[msgId].frames[0].payload[0];
    if (pumpStatus) {
        startPump();
    }
    else {
        stopPump();
    }*/
}

void msg_chargerReceive(uint16_t msgId) {
    uint16_t *payload = g_Messages[msgId].frames[0].payload;

    g_Charger.voltage = ((uint16_t)(payload[0]<<8)) + ((uint16_t)payload[1]);
    g_Charger.current = ((uint16_t)(payload[2]<<8)) + ((uint16_t)payload[3]);
    g_Charger.status = ((uint16_t)payload[4]);
}

void msg_chargerTransmit(uint16_t msgId) {
    uint16_t payload[MAX_FRAME_LENGTH];

    struct Charger maximums;
    charger_compute_maximums(&maximums);

    /*// HACK FIXME DANGER HARDCORE CHARGER
    //payload[0] = 0x0F;//maximums.voltage >> 8;
    //payload[1] = 0x3C; //maximums.voltage & 0xFF;
    payload[0] = 0x10;
    payload[1] = 0x68;
    payload[2] = 0x00;//maximums.current >> 8;
    payload[3] = 0x96;//maximums.current & 0xFF;*/

    payload[0] = maximums.voltage >> 8;
    payload[1] = maximums.voltage & 0xFF;
    payload[2] = maximums.current >> 8;
    payload[3] = maximums.current & 0xFF;
    payload[4] = maximums.status; // Determine whether or not the charger is on or not
    payload[5] = 0;
    payload[6] = 0;
    payload[7] = 0;

    msg_send(MSG_ID_CHARGER_TRANSMIT, MAX_FRAME_LENGTH, payload);
}

void msg_sendErrorStatus(uint16_t msgId) {
    uint16_t payload[8];

    for(int i = 0; i < 8; i++) {
        payload[i] = (uint16_t)((g_error_status >> 8*(7 - i)) & 0xFF);
    }

    //msg_send(MSG_ID_RESP_ERROR_STATUS, 8, (uint16_t*)&g_error_status);
    msg_send(MSG_ID_RESP_ERROR_STATUS, 8, payload);
}

void msg_ReqResetErrorState(uint16_t msgId) {
    clear_all_errors();
}

void msg_ReqPeriodicValues(uint16_t msgId) {
    // payload[0]: 0 = stop sending cvs, 1= start sending cvs
    // Payload[1]: 0 = stop sending temps, 1 = start sending temps
    g_Message_SendPeriodicCvs = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[0];
    g_Message_SendPeriodicCvsFlags = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[1];
    g_Message_SendPeriodicCvsBalance = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[2];
    g_Message_SendPeriodicTemps = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[3];
    g_Message_SendPeriodicTempsFlags = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[4];
    g_Message_SendPeriodicMasterState = g_Messages[MSG_ID_REQ_PERIODIC_VALUES].frames[0].payload[5];
}

void msg_sendPowerLimit(uint16_t msgId) {
    uint16_t powerLimit = calculatePowerLimitKW();

    msg_send(MSG_ID_RESP_POWER_LIMIT, 1, &powerLimit);
}

void msg_sendCvs(uint16_t msgId) {
    if (!g_Message_SendPeriodicCvs) return;

    uint16_t cvs[TOTAL_SLAVES*CELL_VOLTAGES_PER_SLAVE];
    for (int i = 0; i < TOTAL_SLAVES; i++) {
        for (int j = 0; j < CELL_VOLTAGES_PER_SLAVE; j++) {
            cvs[(i*CELL_VOLTAGES_PER_SLAVE)+j] = (uint16_t)convertCvForSending(g_BatteryPack.slave[i].cv[j]);
        }
    }

    msg_send(MSG_ID_RESP_CVS, TOTAL_SLAVES*CELL_VOLTAGES_PER_SLAVE, cvs);
}

void msg_sendCvTemp(uint16_t msgId) { //COMMENTHERE
    uint16_t payload[2];

    payload[0] = (uint16_t)convertCvForSending(g_BatteryPack.averageCv);
    payload[1] = (uint16_t)convertThermistanceToCelsius(g_BatteryPack.min_thermistor);

    msg_send(MSG_ID_RESP_CVTEMP, 2, payload);
}

void msg_sendTSMS(uint16_t msgId) {
    uint16_t tsms = interlockDetect() ? 0xFFFF : 0x0000;
    msg_send(MSG_ID_RESP_TSMS, 0x1, &tsms);
}

void msg_sendTemps(uint16_t msgId) {
    if (!g_Message_SendPeriodicTemps) return;

    uint16_t temps[TOTAL_SLAVES*TEMPERATURE_SENSORS_PER_SLAVE];
    for (int i = 0; i < TOTAL_SLAVES; i++) {
        for (int j = 0; j < TEMPERATURE_SENSORS_PER_SLAVE; j++) {
            temps[(i*TEMPERATURE_SENSORS_PER_SLAVE)+j] = (uint16_t)convertThermistanceToCelsius(g_BatteryPack.slave[i].temperature[j]);
        }
    }

    msg_send(MSG_ID_RESP_TEMPS, TOTAL_SLAVES*TEMPERATURE_SENSORS_PER_SLAVE, temps);
}

uint32_t msg_getMsgId(Message msg) {
    uint32_t retval = 0;

    if (msg.raw_mask != 0) {
        // Message is not part of the messaging protocol and has its own mask (16 first lsb bits)
        retval = (uint32_t)msg.raw_mask;
    }
    else {
        retval |= (uint32_t)((uint32_t)msg.request << 1);
        retval |= (uint32_t)((uint32_t)msg.telemetry << 7);
    }

    retval |= (uint32_t)((uint32_t)msg.id << 16);
    retval |= (uint32_t)((uint32_t)msg.source << 24);

    return retval;
}

// Dynamic payload length
void _msg_sendToUart(uint16_t msgId, int16_t current_frame_idx) {

    int k = 5;
    uint16_t uart_tx[MAX_FRAME_LENGTH+8]; // Frame data (8 bytes) + data length count (1 byte) + Message id (4 bytes) + Start of frame (3 bytes)
    uart_tx[3] = (uint16_t)((g_Messages[msgId].frames[current_frame_idx].id >> 24) & 0xFF);
    uart_tx[2] = (uint16_t)((g_Messages[msgId].frames[current_frame_idx].id >> 16) & 0xFF);
    uart_tx[1] = (uint16_t)((g_Messages[msgId].frames[current_frame_idx].id >> 8) & 0xFF);
    uart_tx[0] = (uint16_t)(g_Messages[msgId].frames[current_frame_idx].id & 0xFF);
    uart_tx[4] = g_Messages[msgId].frames[current_frame_idx].payload_length;
    for (int j = 0; j < g_Messages[msgId].frames[current_frame_idx].payload_length; j++, k++) {
        uart_tx[k] = g_Messages[msgId].frames[current_frame_idx].payload[j];
    }
    uart_tx[k] = 0xFA;
    uart_tx[k+1] = 0xFB;
    uart_tx[k+2] = 0xFC;

    k += 3;

    SCI_writeCharArray(UART_SCI_BASE, (uint16_t *)uart_tx, k);
}

void _msg_sendFrame(uint16_t msgId) {
    g_Messages[msgId].tx_isr_waiting = false;
    int i = g_Messages[msgId].current_frame_idx;
    if (g_Messages[msgId].current_frame_idx < g_Messages[msgId].frame_count) {
        // Output also TX data to UART
        // HACK NO uart
        _msg_sendToUart(msgId, g_Messages[msgId].current_frame_idx);

        CAN_setupMessageObject(MCU_CAN_BASE, msgId, g_Messages[msgId].frames[i].id, CAN_MSG_FRAME_EXT, CAN_MSG_OBJ_TYPE_TX, 0, CAN_MSG_OBJ_TX_INT_ENABLE, g_Messages[msgId].frames[i].payload_length);
        CAN_sendMessage(MCU_CAN_BASE, msgId, g_Messages[msgId].frames[i].payload_length, g_Messages[msgId].frames[i].payload);
    }

    if (g_Messages[msgId].current_frame_idx >= g_Messages[msgId].frame_count -1) {
        g_Messages[msgId].is_sending = 0;
    }
}

void msg_send(uint16_t msgId, uint16_t payloadLen, uint16_t *payload) {
    // XXX: assert if already sending that particular message which indicates overrun
    // should not be an issue with how the protocol is defined

    if (payloadLen != g_Messages[msgId].frame_count) {
        // Message is invalid in length, it will be processed with
        // old data if some is missing as per with the protocol spec
    }

    g_Messages[msgId].current_frame_idx = 0;
    g_Messages[msgId].is_sending = 1;

    // OOPSIE Im chaging array lengths like it's a gender
    //g_Messages[msgId].frame_count = ((payloadLen + 7) / MAX_FRAME_LENGTH);
    //if ((payloadLen % MAX_FRAME_LENGTH) != 0) g_Messages[msgId].frame_count++;

    uint32_t frameId = msg_getMsgId(g_Messages[msgId]);

    //REMOVE: uint16_t payloadDataIdx = 0;

    uint16_t lastFrameIdx;

    for (uint16_t i = 0; i < g_Messages[msgId].frame_count; i++) {
        // Append frame number to id
        g_Messages[msgId].frames[i].id = (uint32_t)((uint32_t)frameId | (uint32_t)(i << 8));
        g_Messages[msgId].frames[i].payload_length = MAX_FRAME_LENGTH;

        // Only for last frame
        //TODO : Move last Frame Iout of loop for
        lastFrameIdx = g_Messages[msgId].frame_count-1;

        if (i == lastFrameIdx) {
            if(g_Messages[msgId].raw_mask == 0)
               g_Messages[msgId].frames[i].id |= 0x1;
            uint16_t plength = payloadLen % MAX_FRAME_LENGTH;
            if (plength == 0) plength = MAX_FRAME_LENGTH;
            g_Messages[msgId].frames[i].payload_length = plength;
        }

        for (int j = 0; j < g_Messages[msgId].frames[i].payload_length; j++) {
            g_Messages[msgId].frames[i].payload[j] = (uint16_t)*(payload+((i*MAX_FRAME_LENGTH)+j));
        }

        //REMOVE: payloadDataIdx += MAX_FRAME_LENGTH;
    }

    // Initiate message transfer
    _msg_sendFrame(msgId);
}

void msg_sendNextFrame(uint16_t msgId) {
    g_Messages[msgId].current_frame_idx++;
    _msg_sendFrame(msgId);

}

void msg_processRx(uint16_t msgId, uint32_t fullMsgId, uint16_t payloadLen, uint16_t *payload)
{
    if(msgId >= G_MESSAGES_LENGTH) {
        return;
    }

    if(g_Messages[msgId].tx) {
        return;
    }

    // Check if received message is part of the messaging protocol
    if (g_Messages[msgId].raw_mask == 0) {
        // Extract: last message bit, message number
        uint16_t msgIdx = (uint16_t)((fullMsgId >> 8) & 0xFF);
        uint16_t lastMsgBit = (uint16_t)(fullMsgId & 0x1);

        if (msgIdx >= g_Messages[msgId].frame_count) {
            // assert oops. The current received message index exceeds frame buffer capacity. Doh!
            return;
        }

        g_Messages[msgId].frames[msgIdx].payload_length = payloadLen;
        g_Messages[msgId].frames[msgIdx].id = fullMsgId;
        for (int i = 0; i < payloadLen; i++) {
            g_Messages[msgId].frames[msgIdx].payload[i] = payload[i];
        }

        // Flag Execution of RX callback if end of message and callback is defined
        if (lastMsgBit == 1 && g_Messages[msgId].func_ptr != 0) {
            g_Messages[msgId].rx_complete_waiting = true;
        }
    }
    else {
        g_Messages[msgId].frames[0].payload_length = payloadLen;
        g_Messages[msgId].frames[0].id = fullMsgId;
        for (int i = 0; i < payloadLen; i++) {
            g_Messages[msgId].frames[0].payload[i] = payload[i];
        }

        if (g_Messages[msgId].func_ptr != 0) {
            g_Messages[msgId].rx_complete_waiting = true;
        }
    }
}

void msg_manage() {
    for (int i = 1; g_Messages[i].id != 0; i++) {
        if (g_Messages[i].is_sending && g_Messages[i].tx_isr_waiting && g_Messages[i].tx_scheduled) {
            msg_sendNextFrame(i);
        }
        else if (g_Messages[i].rx_complete_waiting && g_Messages[i].func_ptr != 0) {
            g_Messages[i].rx_complete_waiting = false;
            g_Messages[i].func_ptr(i);
        }
        else if (g_Messages[i].tx_scheduled && g_Messages[i].func_ptr != 0) {
            g_Messages[i].tx_scheduled = false;
            g_Messages[i].func_ptr(i);
        }
    }
}

void msg_schedule_tx(uint16_t msgId) {
    g_Messages[msgId].tx_scheduled = true;
}
