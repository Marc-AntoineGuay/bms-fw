/*
 * LTC6804.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "LTC6804-1.h"

/*
 * @brief: Reset config stored on master for new slave discharge config
 */
void LTC6804_remove_all_cell_discharge_config(){
    for (uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        g_slave_config[i][4] = CFGR4_INIT;
        g_slave_config[i][5] = CFGR5_INIT;
    }
}

/*!
  @brief Edits and sends new config with updated temperature selection
 */
void LTC6804_start_next_temperature_sensor(uint16_t mux_GPIO_selection) {
    uint16_t new_cfgr0;
    uint16_t GPIO_mode;

    // XXX check if clear AUX is required before new read.
    //LTC6804_send_start_command(CLEAR_REG_AUX);

    // GPIO2 to GPIO5 are mux_selection.
    //Selected bits must be reversed to fit with LTC convention (1 = pull_down_OFF)
    GPIO_mode = (~mux_GPIO_selection) & 0x000F;
    GPIO_mode = (GPIO_mode << 1) | (GPIO_1_MODE); // is on 5 bits

    new_cfgr0 = configByte0(GPIO_mode, REFON, SWTRD, ADCOPT);

    for (uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        g_slave_config[i][0] = new_cfgr0;
    }

}

ErrorFlag LTC6804_send_read_command(uint16_t read_command, LTC6804_Group *data) {
    // Send command to read a register group
    // returns SUCCESS by default

    uint16_t cmd[4];
    uint16_t cmd_pec;
    ErrorFlag spi_error;

    cmd[0] = 0x00;
    cmd[1] = read_command & 0x00FF;

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = cmd_pec >> 8;
    cmd[3] = cmd_pec & 0x00FF;

    wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    spi_write_array(4, cmd);
    spi_error = spi_read_group(data);

    return spi_error;
}

void LTC6804_send_start_command(uint16_t start_command) {
    // Send command to start a function (see LTC6804_StartCommand)
    // returns SUCCESS by default

    uint16_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = start_command >> 8;
    cmd[1] = start_command & 0x00FF;

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = cmd_pec >> 8;
    cmd[3] = cmd_pec & 0x00FF;

    wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

    spi_write_array(4, cmd);
}

/*****************************************************//**
 \brief Write the LTC6804 configuration register
 The configuration is written in descending order so the last device's configuration is written first.
 Config is 6 bytes for each IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array.
 The array should have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |
 ********************************************************/
void LTC6804_write_config(LTC6804_ConfigGroup *config) {
    uint16_t cfg_pec;

    static uint16_t cmd[WRCFG_CMD_LEN];
    uint16_t cmd_index = 0; //command counter

    //1
    // WRCFG command and it's PEC
    cmd[cmd_index++] = 0x00; // Change this is for LTC6804-2 addressing
    cmd[cmd_index++] = WRITE_CONFIG;
    cmd[cmd_index++] = 0x3d; //PEC
    cmd[cmd_index++] = 0x6e;

    //2
    // executes for each LTC6804 in daisy chain, this loops starts with
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (int16_t current_ic = TOTAL_SLAVES - 1; current_ic >= 0; current_ic--) {
        cmd[cmd_index++] = (*config)[current_ic][0];
        cmd[cmd_index++] = (*config)[current_ic][1];
        cmd[cmd_index++] = (*config)[current_ic][2];
        cmd[cmd_index++] = (*config)[current_ic][3];
        cmd[cmd_index++] = (*config)[current_ic][4];
        cmd[cmd_index++] = (*config)[current_ic][5];

        //3
        // calculating the PEC for each ICs configuration register data
        cfg_pec = pec15_calc(6, &(cmd[cmd_index-6]));
        cmd[cmd_index++] = cfg_pec >> 8;
        cmd[cmd_index++] = cfg_pec & 0x00FF;
    }

    //4
    wakeup_idle();

    //5
    spi_write_array(WRCFG_CMD_LEN, cmd);
}

/*!****************************************************
 \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle.
 LTC6804 wakes up when data on pin 42 and pin 41 (CS and SCK) are different (reject common mode noise) for more than 240ns
 Writing garbage will wakeup from low power mode.
 *****************************************************/
void wakeup_idle() {
/*
 *  At minimum, a pair of long isoSPI pulses (–1 and +1) is needed for each device,
 *  separated by more than tREADY or tWAKE (if the core state isSTANDBY or SLEEP, respectively),
 *  but less than tIDLE.
 *  Dummy commands (such as RDCFG) can be executed to generate the long isoSPI pulses.
 */
    // optimize command send: only wakeup idle if g_LTC_last_comm >= 1800ms (2000ms hardware watchdog)
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    // HACK ADD TWO Here
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);
    SPI_writeDataBlockingFIFO(MCU_SPI_BASE, 0xFFFF);

    // wait for FIFO to have something to write in it's buffer
    while(SPI_getTxFIFOStatus(MCU_SPI_BASE) > SPI_FIFO_TX0){}
    // wait for complete FIFO transmission
    while(SPI_getTxFIFOStatus(MCU_SPI_BASE) && SPI_FIFO_TXEMPTY){}

    // 100 us might not be enough in order to wake up 10 slaves. The LTC6804 datasheet states N*tWAKE, this is about 3000us
    DEVICE_DELAY_US(3000);

}

/*!****************************************************
 \brief Wake the LTC6804 from the sleep state

 Generic wakeup commannd to wake the LTC6804 from sleep
 *****************************************************/
void wakeup_sleep() {
}

/*!**********************************************************
 \brief calaculates  and returns the CRC15
 @param[in] uint8_t len: the length of the data array being passed to the function
 @param[in] uint8_t data[] : the array of data that the PEC will be generated from
 @returns The calculated pec15 as an unsigned int
 ***********************************************************/
uint16_t pec15_calc(uint16_t len, //Number of bytes that will be used to calculate a PEC
        uint16_t *data //Array of data that will be used to calculate  a PEC
        ) {
    // XXX replace with Ti's CRC (CRC16-1) and read result in CRC register page 275 http://www.ti.com/lit/ug/spruhs1a/spruhs1a.pdf
    // this might help https://e2e.ti.com/support/microcontrollers/c2000/f/171/t/492359

    uint16_t remainder;
    uint16_t addr;

    remainder = 16; //initialize the PEC
    uint16_t i;
    for (i = 0; i < len; i++) // loops for each byte in data array
            {
        addr = ((remainder >> 7) ^ data[i]) & 0xFF; //calculate PEC table address
        remainder = (remainder << 8) ^ crc15Table[addr];
    }
    return (remainder * 2); //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/*!
 \brief Writes an array of bytes out of the SPI port

 This will only send an even number of bytes

 @param[in] uint8_t len length of the data array being written on the SPI port
 @param[in] uint8_t data[] the data array to be written on the SPI port

 */
void spi_write_array(uint16_t len, uint16_t *data) {
    uint16_t d;
    for (uint16_t i = 0; i < len/2; i++) {
        d = (data[i*2]<<8) | (data[i*2 +1]);
        //SPI_writeDataBlockingFIFO(MCU_SPI_BASE, data[i] << 8);
        SPI_writeDataBlockingFIFO(MCU_SPI_BASE, d);
    }
}


ErrorFlag spi_read_group(LTC6804_Group *data) {
    // read one register group in LTC. Follows a ReadCommand sent.
#warning "find why we must use +1 number of bits for reading. For some reason, WORDS_IN_GROUP_WITH_PEC is 3..."


    ErrorFlag packet_error_check = SUCCESS;
    uint16_t rx_data[TOTAL_SLAVES * WORDS_IN_GROUP_WITH_PEC +SPI_EXTRA_GARBAGE];
    uint16_t pec_received;
    uint16_t pec_calc;


    while(SPI_isBusy(MCU_SPI_BASE)){}
    SPI_resetRxFIFO(MCU_SPI_BASE);
    while(SPI_getRxFIFOStatus(MCU_SPI_BASE) != SPI_FIFO_RXEMPTY){}
    SPI_resetRxFIFO(MCU_SPI_BASE);

    for (uint16_t i = 0; i < (TOTAL_SLAVES * WORDS_IN_GROUP_WITH_PEC +SPI_EXTRA_GARBAGE); i++) {
        SPI_writeDataBlockingFIFO(MCU_SPI_BASE, i);
        //rx_data[i] = SPI_readDataNonBlocking(MCU_SPI_BASE);
        rx_data[i] = SPI_readDataBlockingFIFO(MCU_SPI_BASE);
    }



    // Check PEC for each group received
    // Move read_data from 8 bits words to 16 bits words. Design choice to do this here instead of higher.
    // we receive data from the closest LTC first. Registers from 0 to 5, followed by pec bytes.
    g_EventCount.slave_pec_total++;
    for (uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        // (6*8bits contained in 8*uint16_t) ---> (3*16bits contained in 3*uart16_t)
        uint16_t idx = i * WORDS_IN_GROUP_WITH_PEC +SPI_EXTRA_GARBAGE;

        // for PEC check
        pec_received = rx_data[idx+3];

        // switch byte0 with byte1 in the received data. (LTC transfers LSByte first)
        rx_data[idx+0] = (rx_data[idx+0] << 8) | (rx_data[idx+0] >> 8);
        rx_data[idx+1] = (rx_data[idx+1] << 8) | (rx_data[idx+1] >> 8);
        rx_data[idx+2] = (rx_data[idx+2] << 8) | (rx_data[idx+2] >> 8);
        rx_data[idx+3] = (rx_data[idx+3] << 8) | (rx_data[idx+3] >> 8);


        uint16_t pec_data[BYTES_IN_GROUP];
        pec_data[0] = (rx_data[idx+0] & 0x00FF);
        pec_data[1] = (rx_data[idx+0] >> 8);
        pec_data[2] = (rx_data[idx+1] & 0x00FF);
        pec_data[3] = (rx_data[idx+1] >> 8);
        pec_data[4] = (rx_data[idx+2] & 0x00FF);
        pec_data[5] = (rx_data[idx+2] >> 8);



        // Update with proper 16 bit PEC calculation
        // this is in 8 bits
        pec_calc = pec15_calc(BYTES_IN_GROUP, pec_data); 
        if (pec_calc != pec_received) {
            g_EventCount.slave[i].pec_error++;
            packet_error_check = ERROR_BAD_PEC;
            continue;
        }

        // Packet check confirmed.
        (*data)[i][0] = rx_data[idx+0];
        (*data)[i][1] = rx_data[idx+1];
        (*data)[i][2] = rx_data[idx+2];
    }
    return packet_error_check;
}
