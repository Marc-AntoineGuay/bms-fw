/*
 * memory.c
 *
 *  Created on: Feb 8, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "memory.h"

  /////////////////////
 /// CONFIGURATION ///
/////////////////////

#define M95M02_CMD_WREN                             0x06
#define M95M02_CMD_WRDI                             0x04
#define M95M02_CMD_RDSR                             0x05
#define M95M02_CMD_WRSR                             0x01
#define M95M02_CMD_READ                             0x03
#define M95M02_CMD_WRITE                            0x02
#define M95M02_CMD_READ_ID                          0x83
#define M95M02_CMD_WRITE_ID                         0x82
#define M95M02_CMD_READ_LOCK_STATUS                 0x83
#define M95M02_CMD_LOCK_ID                          0x82

#define M95M02_STATUS_REGISTER_SRWD_BIT             0x80
#define M95M02_STATUS_REGISTER_BP1_BIT              0x08
#define M95M02_STATUS_REGISTER_BP0_BIT              0x04
#define M95M02_STATUS_REGISTER_BP_MASK (M95M02_STATUS_REGISTER_BP1_BIT | M95M02_STATUS_REGISTER_BP0_BIT)
#define M95M02_STATUS_REGISTER_WEL_BIT              0x02
#define M95M02_STATUS_REGISTER_WIP_BIT              0x01

#define M95M02_SEND_ADDRESS_STATE_DONE              0
#define M95M02_SEND_ADDRESS_STATE_BUSY              1

  ////////////////
 /// Helpers ///
//////////////

static void writeSPI(uint16_t data){
    SPI_writeDataBlockingNonFIFO(EEPROM_SPI_BASE, data << 8 );
}

static int getStatus(){
    int value = 0;
    writeSPI(M95M02_CMD_RDSR);

    writeSPI(0xFF);
    SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);
    writeSPI(0xFF); SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);

    writeSPI(0xFF);
    value = SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);

    // delay before next instruction
    while(SPI_isBusy(EEPROM_SPI_BASE)){}
    DEVICE_DELAY_US(50);

    return value;
}

/**
 * Blocking wait for EEPROM status bit conditions before writing
 *
 * Wait for Write Enabled (WEL)
 * Wait until no Write In Progress (WIP)
 */
static inline void waitWelAndWip() {
    while(SPI_isBusy(EEPROM_SPI_BASE)){} // wait until transmission is completed
    DEVICE_DELAY_US(50);
    uint16_t status;
    do {status = getStatus();} while (
        !((status & M95M02_STATUS_REGISTER_WEL_BIT) && !(status & M95M02_STATUS_REGISTER_WIP_BIT))
        || (status == 0xFF));
}

/**
 * Blocking wait for EEPROM status bit conditions before writing
 *
 * Wait until no Write In Progress (WIP)
 */
static inline void waitWip() {
    while(SPI_isBusy(EEPROM_SPI_BASE)){} // wait until transmission is completed
    DEVICE_DELAY_US(50);
    uint16_t status;
    do {status = getStatus();} while ((status & M95M02_STATUS_REGISTER_WIP_BIT) || (status == 0xFF));
}

/**
 * Send WREN to EEPROM. Must be sent once before each write cmd
 */
static inline void sendWriteEnable(void){
    writeSPI(M95M02_CMD_WREN);
}

static inline void sendWriteDisable(void){
    writeSPI(M95M02_CMD_WRDI);
}

static void sendCommandWrite(void){
    waitWip();
    sendWriteEnable();
    // The device then enters a wait state. It waits for the device to be deselected, by Chip Select (S) being driven high.
    waitWelAndWip();
    writeSPI(M95M02_CMD_WRITE);
}

static void sendCommandRead(void){
    waitWip();
    writeSPI(M95M02_CMD_READ);
}


/* Write to a raw adress
static int write(uint32_t address, uint16_t data){
    sendCommandWrite();
    writeSPI((address >> 16) & 0xFF); // MSB
    writeSPI((address >> 8) & 0xFF);
    writeSPI(address & 0xFF); // LSB

    writeSPI(data);
    return 1;
}
*/

/**
 * On ti C2000 microcontrollers
 * sizeof(uint16_t) == 1
 * The ti C2000 has bytes of 16bits
 */
static int writeEepromBuffer(uint32_t address, uint16_t* pBuffer, size_t numWords){
    // Page Write within 10 ms
    ASSERT(((address % 256)+numWords*2) <= 256); // check for page roll-over (Page size: 256 bytes)

    sendCommandWrite();
    writeSPI((address >> 16) & 0xFF); // MSB
    writeSPI((address >> 8) & 0xFF);
    writeSPI(address & 0xFF); // LSB

    uint16_t index;
    for (index = 0; index < numWords; index++)
    {
        writeSPI(pBuffer[index] >> 8); // MSB written first
        writeSPI(pBuffer[index] & 0xFF); // LSB
    }
    while(SPI_isBusy(EEPROM_SPI_BASE)){}
    return index;
}

static int readEepromBuffer(uint32_t address, uint16_t * pBuffer, size_t numWords){
    uint16_t data_MSB, data_LSB;
    ASSERT(((address % 256)+numWords*2) <= 256); // check for page roll-over (Page size: 256 bytes)

    sendCommandRead();
    writeSPI((address >> 16) & 0xFF); // MSB
    writeSPI((address >> 8) & 0xFF);
    writeSPI(address & 0xFF); // LSB

    writeSPI(0xFF);
    SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);
    writeSPI(0xFF);
    SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);

    uint16_t index;
    for (index = 0; index < numWords; index++)
    {
        writeSPI(0xFF);
        data_MSB = SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);
        writeSPI(0xFF);
        data_LSB = SPI_readDataBlockingNonFIFO(EEPROM_SPI_BASE);
        pBuffer[index] = (data_MSB << 8) | data_LSB;
    }
    return index;
}

  /////////////////////
 /// Implementaton ///
/////////////////////

void unprotectEepromMemory(){
    sendWriteEnable();
    waitWelAndWip();
    writeSPI(M95M02_CMD_WRSR);
    writeSPI(0x0000);
}

void protectEepromMemory(){
    sendWriteEnable();
    waitWelAndWip();
    writeSPI(M95M02_CMD_WRSR);
    writeSPI(0x00FF);
}

void loadCheckpointFromEeprom(){
    //TODO ADD PARAM BACK readEepromBuffer(EEPROM_ADDR_PARAM, (uint16_t*)&g_Param, sizeof(g_Param));
    DEVICE_DELAY_US(10000);
    readEepromBuffer(EEPROM_ADDR_STATE_OF_CHARGE, (uint16_t*)&g_BatteryPack.coulomb_counter, sizeof(g_BatteryPack.coulomb_counter));
    DEVICE_DELAY_US(10000);
    readEepromBuffer(EEPROM_ADDR_ERROR_STATUS, (uint16_t*)&g_error_status, sizeof(g_error_status));

    // check if read data is garbage
    if (g_error_status == 0xFFFFFFFFFFFFFFFF){
        // The eeprom returned value 0xFFFF... and this is not normal.
        // XXX send to CAN EEPROM error
        asm(" ESTOP0");
    }
}

void saveCheckpointToEeprom(){
    writeEepromBuffer(EEPROM_ADDR_STATE_OF_CHARGE, (uint16_t*)&(g_BatteryPack.coulomb_counter), sizeof(g_BatteryPack.coulomb_counter));
    //TODO ADD PARAM BACK writeEepromBuffer(EEPROM_ADDR_PARAM, (uint16_t*)&g_Param, sizeof(g_Param));

    // save_error_status() is only called when there is an error, before killing the car.
    g_saveDataEepromSeconds= 0;
}

void saveErrorStatus(){
    // MCU's last words :(
    writeEepromBuffer(EEPROM_ADDR_ERROR_STATUS, (uint16_t*)&g_error_status, sizeof(g_error_status));
}
