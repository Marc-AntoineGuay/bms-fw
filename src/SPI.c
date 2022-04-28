/*
 * SPI.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "SPI.h"

void init_SPI(){
    init_ltc_SPI();
    init_eeprom_SPI();
}

void init_ltc_SPI(){
    // Using SPI_A for EEPROM, SPI_B for Slaves
    // All SPI to slave have external pull-ups
    // GPIO16 is the SPISIMOA.

    GPIO_setMasterCore(GPIO_MCU_SPI_MOSI, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_SPI_MOSI);
    GPIO_setPadConfig(GPIO_MCU_SPI_MOSI, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_SPI_MOSI, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_MCU_SPI_MISO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_SPI_MISO);
    GPIO_setPadConfig(GPIO_MCU_SPI_MISO, GPIO_PIN_TYPE_STD); // External pull-up. Use 1kohm for LTC6804
    GPIO_setQualificationMode(GPIO_MCU_SPI_MISO, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_MCU_SPI_SCK, GPIO_CORE_CPU1);
    GPIO_setPinConfig( PIN_MAP_MCU_SPI_SCK);
    GPIO_setPadConfig(GPIO_MCU_SPI_SCK, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_SPI_SCK, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_MCU_SPI_CS, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_MCU_SPI_CS);
    GPIO_setPadConfig(GPIO_MCU_SPI_CS, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(GPIO_MCU_SPI_CS, GPIO_QUAL_ASYNC);

    // Must put SPI into reset before configuring it
    SPI_disableModule(MCU_SPI_BASE);

    // with default clock, data_rate is valid if ASSERT((lspclkHz / bitRate) <= 128U);
    // which is a maximum of 0.3906MHz (400kHz is good)
    // the TI driver values are inverted for PHA, if we follow the CPOL and CPHA protocol (Motorola SPI Block Guide)
    // LTC needs CPOL=1 CPHA=1, which is SPI_PROT_POL1_PHA0
    SPI_setConfig(MCU_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, 500000, 16);

    SPI_setSTESignalPolarity(MCU_SPI_BASE, SPI_STE_ACTIVE_LOW);

    //SPI_enableLoopback(SPIA_BASE);
    //SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_disableLoopback(MCU_SPI_BASE);
    SPI_enableFIFO(MCU_SPI_BASE);

    // Configuration complete. Enable the module.
    SPI_enableModule(MCU_SPI_BASE);
}

void init_eeprom_SPI(){

    GPIO_setMasterCore(GPIO_EEPROM_SPI_MOSI, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_EEPROM_SPI_MOSI);
    GPIO_setPadConfig(GPIO_EEPROM_SPI_MOSI, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_EEPROM_SPI_MOSI, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_EEPROM_SPI_MISO, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_EEPROM_SPI_MISO);
    GPIO_setPadConfig(GPIO_EEPROM_SPI_MISO, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_EEPROM_SPI_MISO, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_EEPROM_SPI_SCK, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_EEPROM_SPI_SCK);
    GPIO_setPadConfig(GPIO_EEPROM_SPI_SCK, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_EEPROM_SPI_SCK, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO_EEPROM_SPI_CS, GPIO_CORE_CPU1);
    GPIO_setPinConfig(PIN_MAP_EEPROM_SPI_CS);
    GPIO_setPadConfig(GPIO_EEPROM_SPI_CS, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(GPIO_EEPROM_SPI_CS, GPIO_QUAL_ASYNC);

    SPI_disableModule(EEPROM_SPI_BASE);

    SPI_setConfig(EEPROM_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA0,
                  SPI_MODE_MASTER, 1000000, 8);


    SPI_enableModule(EEPROM_SPI_BASE);

}
