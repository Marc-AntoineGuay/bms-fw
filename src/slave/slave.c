/*
 * slave.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Louis-Philippe Asselin
 *
 *  Higher level interface for LTC6804-1 slave devices
 *
 */

#include "slave.h"

LTC6804_ConfigGroup g_slave_config;

/*
 * @brief: Read the value of the measured cell voltages on each slave
 *
 * Requires the measurement to be completed.
*/
void slave_read_all_CV() {
    // Depending on the priority of the measurements (ADC gpio vs cv), might not be done yet.
    // Maybe reset the values before start measurements and reread later if zero?
    g_EventCount.slave_cv_total++;
    slave_read_data(READ_CV_A);
    slave_read_data(READ_CV_B);
    slave_read_data(READ_CV_C);
    slave_read_data(READ_CV_D);
}

/*!
 * @brief: performs checks and tests on slave devices.
 */
void slave_test() {
    // HACK No self test
    return;

    if (LTC6804_test_cell_voltage() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_SELF_TEST_CV);
        asm(" ESTOP0");
    }
    if (LTC6804_test_GPIO() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_SELF_TEST_GPIO);
        asm(" ESTOP0");
    }
    if (LTC6804_test_status_register() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_SELF_TEST_STAT);
        asm(" ESTOP0");
    }
    if (LTC6804_test_clear_all() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_CLEAR_TEST);
        asm(" ESTOP0");
    }
    if (LTC6804_test_internal_diagnostic() != SUCCESS) {
        asm(" ESTOP0");
    }
    if (LTC6804_test_open_wire() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_OPEN_WIRE);
        asm(" ESTOP0");
    }
    if (LTC6804_test_accuracy_with_vref2() != SUCCESS) {
        set_error_bit(ERROR_SLAVE_ACCURACY);
        asm(" ESTOP0");
    }
}

/*!
 * @brief: writes config from master to the slaves.
 *
 * Sent config is global slave config.
 */
void slave_write_current_config(){
    LTC6804_write_config(&g_slave_config);
}

/*!
 * @brief: This function initializes all slaves and performs health tests on the LTC6804.
 */
void slave_initialize() {
    // write configurations to all slaves
    for (uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        g_slave_config[i][0] = CFGR0_INIT;
        g_slave_config[i][1] = CFGR1_INIT;
        g_slave_config[i][2] = CFGR2_INIT;
        g_slave_config[i][3] = CFGR3_INIT;
        g_slave_config[i][4] = CFGR4_INIT;
        g_slave_config[i][5] = CFGR5_INIT;
    }
    LTC6804_write_config(&g_slave_config);
}

void slave_read_config() {
    slave_read_data(READ_CONFIG);
}

/*!
 * @brief: Read the value of the measured temperature sensor on each slave
 *
 * Requires the measurement to be completed.
*/
void slave_read_temperature_sensor() {
    // Send depending on the priority of the measurements (ADC gpio vs cv), might not be done yet.
    // Maybe reset the values before start measurements and reread later if zero?
    slave_read_data(READ_AUX_A);
    slave_read_data(READ_AUX_B);
}

/**
 * @brief Primary function to read and update Slave data
 * Calls appropriate functions to read, check and update a data payload from all LTC6804 in the chain
 */
ErrorFlag slave_read_data(LTC6804_ReadCommand read_command) {
    LTC6804_Group new_data; // Watch out, if using interrupts below, this will cause problems
    ErrorFlag error_status = SUCCESS;

    error_status = LTC6804_send_read_command(read_command, &new_data);
    if (error_status != SUCCESS) {
        return error_status;
    }

    error_status = slave_check_and_update_bms_data(read_command, &new_data);

    return error_status;
}

/*!
 * @brief Start measuring next temperature sensor on each slave
 *
 * Round robin reading of each 16 temperature sensors on each slaves.
 *
 * Temperature sensors are muxed with GPIO outputs of the slaves' LTC6804.
 * This function sends a new configuration for each LTC6804, which sets the mux to the corrresponding temperature sensor.
 */
void slave_start_next_temperature_sensor() {
    g_temperatureSensorIdx = (g_temperatureSensorIdx + 1) % 16;
    LTC6804_start_next_temperature_sensor(g_temperatureSensorIdx);
}

/*
 *  @brief Send Start Measure Cell Voltages of all cells
 */
void start_measure_CV_ADC() {
    uint16_t command;
    LTC6804_send_start_command(CLEAR_REG_CV);
    command = format_command_ADCVAX(ADC_MODE_FILTERED, DISCHARGE_PERMITTED);
    LTC6804_send_start_command(command);
}

/*
 * @brief: Read CV and current Temperature Sensor from all slaves
 *
 * Read cell voltages measured and current temperature sensor on all slaves.
 * Restarts measurements and resets counter
 */
void slave_read_CV_and_temperature(){
    // Should send measure GPIO before CV because it reconfigures the LTC
    slave_read_all_CV();

    slave_read_temperature_sensor();

    slave_start_next_temperature_sensor();

    slave_write_current_config();

    start_measure_CV_ADC();
    slave_start_measure_GPIO_ADC();

    g_slaveMeasurementCountMs = 0;
}

/*
 * @brief Send Start Measure temperature sensor
 */
void slave_start_measure_GPIO_ADC() {
    uint16_t command;
    LTC6804_send_start_command(CLEAR_REG_AUX);
    command = format_command_ADAX(ADC_MODE_FILTERED, AUX_CH_ALL);
    LTC6804_send_start_command(command);
}

/*!
 * @brief: Update config for new slave discharge config
 *
 * Does not send the new config to the slaves.
 */
void slave_update_cell_discharge_config(uint16_t slave_number, uint16_t discharge_config){
    g_slave_config[slave_number][4] = configByte4(discharge_config);
    g_slave_config[slave_number][5] = configByte5(CELL_DISCHARGE_TIME, discharge_config);
}


