/*
 * SCI.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "SCI.h"

void init_SCI()
{
    // SCI GPIO config
    GPIO_setMasterCore(GPIO_MCU_UART_RX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_UART_RX);
    GPIO_setDirectionMode(GPIO_MCU_UART_RX, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(GPIO_MCU_UART_RX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_UART_RX, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_MCU_UART_TX, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_UART_TX);
    GPIO_setDirectionMode(GPIO_MCU_UART_TX, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(GPIO_MCU_UART_TX, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_UART_TX, GPIO_QUAL_ASYNC);

    // Initialize SCI and its FIFO.
    SCI_performSoftwareReset(UART_SCI_BASE);

    // Configure SCI
    SCI_setConfig(
            UART_SCI_BASE, DEVICE_LSPCLK_FREQ, 115200,
            (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(UART_SCI_BASE);
    SCI_resetRxFIFO(UART_SCI_BASE);
    SCI_resetTxFIFO(UART_SCI_BASE);
    SCI_clearInterruptStatus(UART_SCI_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableFIFO(UART_SCI_BASE);
    SCI_enableModule(UART_SCI_BASE);
    SCI_performSoftwareReset(UART_SCI_BASE);

}

/** Brief: prints char array to SCI
 */
void print(char *str){
    uint16_t length = 0;
    while(str[length] != '\0' ) {
        length += 1;
    }
    SCI_writeCharArray(UART_SCI_BASE, (uint16_t*) str, length);
}

/** Brief: Converts and prints a single uint16_t to hexadecimal
 */
void print_hex(uint16_t data)
{
    char data_hex[4];
    uint16_t mask;
    uint16_t letter_num;
    char letter_char;

    for (uint16_t i = 0; i<4; i++){
        mask = 0x000F << (4*i);
        letter_num = (data & mask) >> (4*i);

        if (letter_num >= 0x0A){
            letter_char = letter_num + 55;
        }
        else {
            letter_char = letter_num + 48;
        }
        data_hex[3-i] = letter_char;

    }

    print(&data_hex[0]);
}


