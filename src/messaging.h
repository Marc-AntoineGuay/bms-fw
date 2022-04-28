/*
 * power_limit.h
 *
 *  Created on: 9 avr. 2018
 *      Author: Nicolas Juteau
 */

#ifndef SRC_MESSAGING_H_
#define SRC_MESSAGING_H_
#include <device.h>
#include "defines.h"
#include "util.h"
#include "charger.h"
#include "hardware.h"
#include "power_limit.h"
#include "error_management.h"
#include "util.h"
#include "hardware.h"

/*
 * A little bit of definitions:
 * - Message: a message that is considered complete as a whole, separated by packets (i.e: application layer)
 * - Packet: components of a message, added together they form the message (i.e: transport layer)
 *
 * - Mailbox: specific to CANbus, a mailbox is a ressource dedicated to a specific message id or a general message id (whose some bits are masked)
 */

#define MAX_FRAME_LENGTH           8


// Sources
#define MSG_SOURCE_TELEMETRY_ECU_CMDS   0x01
#define MSG_SOURCE_MASTER_BMS           0x02
#define MSG_SOURCE_INVERTERS            0x04
#define MSG_SOURCE_SENSING_SENSORS      0x08
#define MSG_SOURCE_CHARGER              0x18

// Sources using request
#define MSG_SOURCE_REQ_ECU_DASH_TELEMETRY_MASTER_CAN_NODE   0x02
#define MSG_SOURCE_REQ_BMS_ECU                              0x02
#define MSG_SOURCE_REQ_INVERTERS_ECU                        0x04
#define MSG_SOURCE_REQ_ECU_DASH_NODES_TELEMTRY_INVERTERS    0x40

// Message IDs (protocol)
#define MSG_ID_RESP_POWER_LIMIT         0x01
#define MSG_ID_RESP_CVS                 0x02
#define MSG_ID_RESP_TEMPS               0x03
#define MSG_ID_REQ_PERIODIC_VALUES      0x04
#define MSG_ID_REQ_RESET_ERROR_STATE    0x05
#define MSG_ID_CHARGER_TRANSMIT         0x06 // MANDATORY: this msg id must absolutely be 0x06
#define MSG_ID_RESP_ERROR_STATUS        0x07
#define MSG_ID_REQ_MANAGE_PUMP          0x08
#define MSG_ID_RESP_MASTERBMS_STATE     0x09
#define MSG_ID_RESP_CVS_FLAGS           0x0A
#define MSG_ID_RESP_TEMPS_FLAGS         0x0B
#define MSG_ID_RESP_CVS_BALANCE         0x0C
#define MSG_ID_RESP_TSMS                0x0D
#define MSG_ID_RESP_CVTEMP              0x0E //COMMENTHERE
#define MSG_ID_CHARGER_RECEIVE          0xFF // MANDATORY: this msg id must be absolutely 0xFF

// Raw masks for messages that are not part of the messaging protocol
// (16 first lsb bits of the message id)
#define MSG_RAW_MASK_CHARGER_TRANSMIT   0xE5F4
#define MSG_RAW_MASK_CHARGER_RECEIVE    0x50E5

// Mask set on RX mailboxes so the CAN module filter out received messages id associated to it
// 0 = don't care, 1 = care
#define MSG_MASK_RX                     0b11111111111110000000000000000
#define MSG_MASK_RX_RAW                 0b11111111111111111111111111111

// Structures
typedef struct {
    uint32_t id; // Full frame id, built from the message information (source, id, message_num, mask (telemetry, request, message_end))
    uint16_t payload_length; // Number of elements in payload
    uint16_t payload[MAX_FRAME_LENGTH];
} Frame;

typedef struct {
    const uint16_t id;
    const uint16_t source;
    uint16_t frame_count; // number of frame expected in a received message / to be transmitted
    const bool telemetry; // denotes reproduced message by CAN node / mark messages that must be sent over by telemetry
    const bool request;
    const bool tx;
    bool tx_isr_waiting;
    bool rx_complete_waiting;
    bool is_sending;
    bool tx_scheduled;
    Frame *frames; //payload[MAX_MESSAGE_FRAME_LENGTH];
    uint16_t current_frame_idx;
    void (*func_ptr)(uint16_t); // Callback function called once a whole message was received and reconstructed or when a scheduled tx is to be sent
    uint16_t raw_mask; // For messages that are not part of the messaging protocol, it might be necessary to define a custom mask that will override the first 16 lsb bits of the message id

} Message;

extern Message g_Messages[];

// Frame buffers
extern Frame g_Message_Resp_PowerLimit_Fb[];
extern Frame g_Message_Resp_Cvs_Fb[];
extern Frame g_Message_Resp_Temps_Fb[];
extern Frame g_Message_Req_PeriodicValues_Fb[];
extern Frame g_Message_Req_ResetErrorState_Fb[];
extern Frame g_Message_Resp_ErrorStatus_Fb[];
extern Frame g_Message_ChargerTransmit_Fb[];
extern Frame g_Message_ChargerReceive_Fb[];
extern Frame g_Message_Req_ManagePump_Fb[];
extern Frame g_Message_Resp_MasterBMSState_Fb[];
extern Frame g_Message_Resp_Cvs_Flags_Fb[];
extern Frame g_Message_Resp_Temps_Flags_Fb[];
extern Frame g_Message_Resp_Cvs_Balance_Fb[];
extern Frame g_Message_Resp_TSMS[];
extern Frame g_Message_Resp_CvTemp_Fb[]; //COMMENTHERE

// Global variables
extern bool g_Message_SendPeriodicCvs;
extern bool g_Message_SendPeriodicTemps;

// Function prototypes
uint32_t msg_getMsgId(Message msg);

void msg_manage();
void msg_send(uint16_t msgId, uint16_t payloadLen, uint16_t *payload);
void msg_sendNextFrame(uint16_t msgId);
void msg_processRx(uint16_t msgId, uint32_t fullMsgId, uint16_t payloadLen, uint16_t *payload);

void msg_ReqPeriodicValues(uint16_t msgId);
void msg_ReqResetErrorState(uint16_t msgId);
void msg_ReqManagePump(uint16_t msgId);

void msg_sendCvs(uint16_t msgId);
void msg_sendTemps(uint16_t msgId);
void msg_sendPowerLimit(uint16_t msgId);
void msg_sendErrorStatus(uint16_t msgId);
void msg_chargerTransmit(uint16_t msgId);
void msg_chargerReceive(uint16_t msgId);
void msg_sendMasterBMSState(uint16_t msgId);
void msg_sendCvsFlags(uint16_t msgId);
void msg_sendTempsFlags(uint16_t msgId);
void msg_sendCvsBalance(uint16_t msgId);
void msg_sendTSMS(uint16_t msgId);
void msg_sendCvTemp(uint16_t msgId); //COMMENTHERE

void msg_schedule_tx(uint16_t msgId);

#endif /* SRC_MESSAGING_H_ */
