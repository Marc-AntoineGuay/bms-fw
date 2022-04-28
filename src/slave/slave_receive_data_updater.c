/*
 * slave_receive_data_updater.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "slave.h"
#include "../globals.h"

uint16_t voltageCounter[TOTAL_SLAVES][CELL_VOLTAGES_PER_SLAVE]; // Counter for bad voltages

uint16_t voltageFilter(uint16_t oldVoltage, uint16_t newVoltage, uint16_t *errorCounter)
{

    /*if (*errorCounter >= 15)
    {
        return newVoltage;
    }

    if (newVoltage > 25000 && newVoltage < 42500)
    {
        *errorCounter = 0;

        if ((newVoltage + 100 > oldVoltage &&
              newVoltage - 100 < oldVoltage) ||
             (oldVoltage > g_BatteryPack.averageCv * 1.1f ||
              oldVoltage < g_BatteryPack.averageCv * 0.9f))
        {
            return newVoltage;
        }
        else
        {
            return oldVoltage;
        }

    }
    else
    {
        *errorCounter++;

        if (oldVoltage > 25000 && oldVoltage < 42500)
        {
            return oldVoltage;
        }
        else
        {
            return g_BatteryPack.averageCv;
        }
    }*/
    return newVoltage;
}

/**
 * @brief Checks for error flags and save newly received data
 *
 * This function is called for every single read_command.
 */
ErrorFlag slave_check_and_update_bms_data(LTC6804_ReadCommand read_command, LTC6804_Group *data) {
    switch(read_command) {
    case READ_CV_A: // Cell Voltages 1, 2, 3 (LTC Datasheet) are indexes 0, 1, 2
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            g_BatteryPack.slave[i].cv[0] = voltageFilter(g_BatteryPack.slave[i].cv[0], (*data)[i][0], &voltageCounter[i][0]);
            g_BatteryPack.slave[i].cv[1] = voltageFilter(g_BatteryPack.slave[i].cv[1], (*data)[i][1], &voltageCounter[i][1]);
            g_BatteryPack.slave[i].cv[2] = voltageFilter(g_BatteryPack.slave[i].cv[2], (*data)[i][2], &voltageCounter[i][2]);
            g_EventCount.slave[i].cv_A_read++;
        }
        break;
    case READ_CV_B: // Cell Voltages 4, 5, 6
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            g_BatteryPack.slave[i].cv[3] = voltageFilter(g_BatteryPack.slave[i].cv[3], (*data)[i][0], &voltageCounter[i][3]);
            g_BatteryPack.slave[i].cv[4] = voltageFilter(g_BatteryPack.slave[i].cv[4], (*data)[i][1], &voltageCounter[i][4]);
            g_BatteryPack.slave[i].cv[5] = voltageFilter(g_BatteryPack.slave[i].cv[5], (*data)[i][2], &voltageCounter[i][5]);
            g_EventCount.slave[i].cv_B_read++;
        }
        break;
    case READ_CV_C: // Cell Voltages 7, 8, 9
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            g_BatteryPack.slave[i].cv[6] = voltageFilter(g_BatteryPack.slave[i].cv[6], (*data)[i][0], &voltageCounter[i][6]);
            g_BatteryPack.slave[i].cv[7] = voltageFilter(g_BatteryPack.slave[i].cv[7], (*data)[i][1], &voltageCounter[i][7]);
            g_BatteryPack.slave[i].cv[8] = voltageFilter(g_BatteryPack.slave[i].cv[8], (*data)[i][2], &voltageCounter[i][8]);
            g_EventCount.slave[i].cv_C_read++;
        }
        break;
    case READ_CV_D: // Cell Voltages 10, 11, 12
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            /*if (i == 5)
            {
                g_BatteryPack.slave[i].cv[9] = g_BatteryPack.averageCv;
            }
            else
            {
                g_BatteryPack.slave[i].cv[9] = voltageFilter(g_BatteryPack.slave[i].cv[9], (*data)[i][0], voltageCounter[i][9]);
            }*/
            g_BatteryPack.slave[i].cv[9] = voltageFilter(g_BatteryPack.slave[i].cv[9], (*data)[i][0], &voltageCounter[i][9]);
            // g_BatteryPack.slave[i].cv[10] = (*data)[i][1];
            // g_BatteryPack.slave[i].cv[11] = (*data)[i][2];
            // CV_11 and CV_12 are not connected to cells. Only LTC6804 ADC inputs [0,9] are connected to cells.
            g_EventCount.slave[i].cv_D_read++;
        }
        break;
    case READ_AUX_A: // GPIO measurements 1, 2, 3
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            // GPIO_1 is temperature measurement for current temperature sensor
            g_BatteryPack.slave[i].temperature[g_temperatureSensorIdx] = (*data)[i][0];
            g_EventCount.slave[i].temperature_read[g_temperatureSensorIdx]++;
        }
        break;
    case READ_AUX_B: // GPIO measurements 4, 5 and REF (2nd reference voltage)
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            g_BatteryPack.slave[i].vref2 = (*data)[i][2]; // VREF2
        }
        break;

    case READ_STAT_A: // SOC, ITMP, VA
        // it is not required to save this data
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            // uint16_t SumAllCells = (*data)[i][0] * 20; // in 100uV

            float itmp;
            itmp = (float) ((*data)[i][1]);
            g_BatteryPack.slave[i].internalDieTemperature = itmp * 0.0133 - 273 ; // InternalDieTemperature (°C) = (ITMP)*100µV/(7.5mV)°C – 273°C

            // uint16_t va = (*data)[i][2];
            // Analog power supply measurement (VREG) = VA • 100µV
        }
        break;
    case READ_STAT_B: // VD, Cells Overvoltages, Cells Undervoltages, THSD, MUXFAIL, RSVD, REV
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            // Digital power supply measurement (VREGD) = VD • 100µV
            g_BatteryPack.slave[i].vd =(*data)[i][0];

            // uint16_t cells_OV_or_UV_C00_to_C9 = (*data)[i][1];
            // uint16_t cells_OV_or_UV_C10_to_C12 = ((*data)[i][2]) & 0x00FF;

            bool muxfail = ((*data)[i][2]) & 0x0200; // MUXFAIL
            g_BatteryPack.slave[i].muxfail = muxfail;

            // check for Thermal Shutdown Status and raise warning. (THSD) bit cleared to 0 on read of status register group B
            // THSD is set to 1 after CLRSTAT command (used to check if the register works)
            bool thsd = ((*data)[i][2]) & 0x0100; // THSD
            g_BatteryPack.slave[i].internalDieThermalShutdown = thsd;
        }
        break;

    case READ_CONFIG: // Config read from device
        for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
            //Used for debugging purposes only.
            // g_BatteryPack.slave[i].readConfig[0] = (*data)[i][0];
            // g_BatteryPack.slave[i].readConfig[1] = (*data)[i][1];
            // g_BatteryPack.slave[i].readConfig[2] = (*data)[i][2];
        }
        break;
    }

    return SUCCESS;
}
