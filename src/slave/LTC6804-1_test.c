/*
 * device_test.c
 *
 *  Created on: Mar 31, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "LTC6804-1_test.h"

uint16_t open_wire_cell_number; // Problematic cell number

// returns 1 if data received is not the same for at least one slave
bool _compare_read_data(LTC6804_Group *data, uint16_t cmp){
    for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        for (uint16_t j = 0; j < WORDS_IN_GROUP; j++){
            if ((*data)[i][0] != cmp) {
                return 1;
            }
        }
    }

    return 0;
}

// returns 1 if data received is not the same for at least one slave
// returns 1 if error on transmission or reception of command and data
bool _read_and_compare_test_result(uint16_t read_command, uint16_t cmp){
    LTC6804_Group new_data;
    ErrorFlag error_status = SUCCESS;

    error_status = LTC6804_send_read_command(read_command, &new_data);
    if (error_status != SUCCESS) {
        return 1;
    }

    return _compare_read_data(&new_data, cmp);
}

/*
 * @brief: Checks for open wire on any of the slave's cells.
 *
 * We use 10 cells on each slave. [C0, C10]
 * C10 is connected to C0 on the next slave.
 *
 * Function exits on the first open wire detected.
 * Sets open_wire_cell_number to the index of the problematic cell
 * Index is in [0, TOTAL_SLAVES*10]
 * Index starts on first cell of the first slave in the daisy chain.
 * if index is the first cell of a slave, C0 is open.
 * if index is the last cell of a slave, C12 is open.
 * See LTC6804 datasheet Open-Wire Check (ADOW Command)
 *
 * Returns FAIL on the first open wire detected.
 * Returns SUCCESS if all is good.
 */
ErrorFlag LTC6804_test_open_wire() {
    // ADOW command
    //used to check for any open wires between the ADCs in the LTC6804 and the external cells
    // WARNING: index values for cell arrays in code are from [0, 12[
    // index values for cells are [1, 12] in datasheet
    uint16_t cmd_adow;
    uint16_t cell_pull_up[CELL_VOLTAGES_PER_SLAVE*TOTAL_SLAVES];
    uint16_t cell_pull_down[CELL_VOLTAGES_PER_SLAVE*TOTAL_SLAVES];
    int32_t cell_delta, cell_pu, cell_pd;

    // 1)
    // Send command at least twice with PUP=1
    cmd_adow = format_command_ADOW(ADC_MODE_FILTERED, DISCHARGE_NOT_PERMITTED, CELL_CH_ALL, 1U);
    LTC6804_send_start_command(cmd_adow);
    DEVICE_DELAY_US(250000);
    LTC6804_send_start_command(cmd_adow);
    DEVICE_DELAY_US(250000);
    // Read all CV and save all pull-up readings in array
    slave_read_all_CV();
    for (uint16_t slv=0; slv<TOTAL_SLAVES; slv++){
        uint16_t slv_start_idx = slv*CELL_VOLTAGES_PER_SLAVE;
        for(uint16_t cv=0; cv<CELL_VOLTAGES_PER_SLAVE; cv++){
            cell_pull_up[slv_start_idx + cv] = g_BatteryPack.slave[slv].cv[cv];
        }
    }

    // 2)
    // Send command at least twice with PUP=0
    cmd_adow = format_command_ADOW(ADC_MODE_FILTERED, DISCHARGE_NOT_PERMITTED, CELL_CH_ALL, 0);
    LTC6804_send_start_command(cmd_adow);
    DEVICE_DELAY_US(250000);
    LTC6804_send_start_command(cmd_adow);
    DEVICE_DELAY_US(250000);
    // Read all CV and save all pull-down readings in array
    slave_read_all_CV();
    for (uint16_t slv=0; slv<TOTAL_SLAVES; slv++){
        uint16_t slv_start_idx = slv*CELL_VOLTAGES_PER_SLAVE;
        for(uint16_t cv=0; cv<CELL_VOLTAGES_PER_SLAVE; cv++){
            cell_pull_down[slv_start_idx + cv] = g_BatteryPack.slave[slv].cv[cv];
        }
    }

    // 3)
    // cell_delta = cell_pull_up - cell_pull_down;
    // for all cells except first and last
    // 4)
    // For all values of cells from 1 to 11
    // if cell_1 = 0.0000 then C0 is open (first cell)
    // if cell_LAST= 0.0000 then CLAST is open. (LAST=CELL_VOLTAGES_PER_SLAVE)
    // index of cell_1 is 0. index of cell_last is CELL_VOLTAGES_PER_SLAVE-1
    for (uint16_t slv=0; slv<TOTAL_SLAVES; slv++) {
        uint16_t slv_first_cell_idx = slv*CELL_VOLTAGES_PER_SLAVE;
        uint16_t slv_last_cell_idx = slv_first_cell_idx + CELL_VOLTAGES_PER_SLAVE-1;
        // check cell_1 for C0 open connection
        if (cell_pull_up[slv_first_cell_idx] == 0) {
            set_error_bit(ERROR_SLAVE_OPEN_WIRE);
            open_wire_cell_number = slv_first_cell_idx;
            return FAIL;
        }
        // check cell_LAST for CLAST open connection
        else if (cell_pull_up[slv_last_cell_idx] == 0) {
            set_error_bit(ERROR_SLAVE_OPEN_WIRE);
            open_wire_cell_number = slv_last_cell_idx;
            return FAIL;
        }
    }

    // if cell_delta < -400mV, cell is open.
    // valid for cells index in [2, 12[, which is is [1, 10] in code
    for (uint16_t i=0; i<(TOTAL_SLAVES*CELL_VOLTAGES_PER_SLAVE); i++){
        bool is_first = (i+1) % CELL_VOLTAGES_PER_SLAVE == 0;
        bool is_last = i % CELL_VOLTAGES_PER_SLAVE == 0;

        // skip first and last. Their checks are done above.
        if (is_first || is_last) continue;

        else {
            cell_pu = (int32_t) cell_pull_up[i];
            cell_pd = (int32_t) cell_pull_down[i];
            cell_delta = cell_pu - cell_pd;
            // at this point, cell_delta is in 100uV.
            // check if cell_delta < -400mV == 4000*100uV
            if (cell_delta < -4000) {
                set_error_bit(ERROR_SLAVE_OPEN_WIRE);
                open_wire_cell_number = i;
                return FAIL;
            }
        }
    }

    return SUCCESS;
}

ErrorFlag LTC6804_test_cell_voltage() {
    // CVST command (self test cell voltage)
    uint16_t cmd;
    bool cmp_A, cmp_B, cmp_C, cmp_D;

    // test with self test mode #1
    cmd = format_command_self_test(START_SELF_TEST_CV, ADC_MODE_NORMAL, SELF_TEST_1);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_CV_A, 0x9555);
    cmp_B =_read_and_compare_test_result(READ_CV_B, 0x9555);
    cmp_C =_read_and_compare_test_result(READ_CV_C, 0x9555);
    cmp_D =_read_and_compare_test_result(READ_CV_D, 0x9555);
    if (cmp_A || cmp_B || cmp_C || cmp_D){
        set_error_bit(ERROR_SLAVE_SELF_TEST_CV);
        return FAIL;
    }

    // test with self test mode #2
    cmd = format_command_self_test(START_SELF_TEST_CV, ADC_MODE_NORMAL, SELF_TEST_2);
//    LTC6804_send_start_command(CLEAR_REG_CV);
//    DEVICE_DELAY_US(5000);
//    cmp_A =_read_and_compare_test_result(READ_CV_A, 0xFFFF);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_CV_A, 0x6AAA);
    cmp_B =_read_and_compare_test_result(READ_CV_B, 0x6AAA);
    cmp_C =_read_and_compare_test_result(READ_CV_C, 0x6AAA);
    cmp_D =_read_and_compare_test_result(READ_CV_D, 0x6AAA);
    if (cmp_A || cmp_B || cmp_C || cmp_D){
        set_error_bit(ERROR_SLAVE_SELF_TEST_CV);
        return FAIL;
    }

    return SUCCESS;
}

ErrorFlag LTC6804_test_GPIO() {
    // AXST command (self test GPIO)
    uint16_t cmd;
    bool cmp_A, cmp_B;

    // test with self test mode #1
    cmd = format_command_self_test(START_SELF_TEST_GPIO, ADC_MODE_NORMAL, SELF_TEST_1);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_AUX_A, 0x9555);
    cmp_B =_read_and_compare_test_result(READ_AUX_B, 0x9555);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_SELF_TEST_GPIO);
        return FAIL;
    }

    // test with self test mode #2
    cmd = format_command_self_test(START_SELF_TEST_GPIO, ADC_MODE_NORMAL, SELF_TEST_2);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_AUX_A, 0x6AAA);
    cmp_B =_read_and_compare_test_result(READ_AUX_B, 0x6AAA);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_SELF_TEST_GPIO);
        return FAIL;
    }

    return SUCCESS;
}

ErrorFlag LTC6804_test_status_register() {
    //STATST command (self test)
    uint16_t cmd;
    bool cmp_A, cmp_B;

    // test with self test mode #1
    cmd = format_command_self_test(START_SELF_TEST_STAT, ADC_MODE_NORMAL, SELF_TEST_1);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_STAT_A, 0x9555);
    cmp_B =_read_and_compare_test_result(READ_STAT_B, 0x9555);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_SELF_TEST_STAT);
        return FAIL;
    }

    // test with self test mode #2
    cmd = format_command_self_test(START_SELF_TEST_STAT, ADC_MODE_NORMAL, SELF_TEST_2);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(5000);
    cmp_A =_read_and_compare_test_result(READ_STAT_A, 0x6AAA);
    cmp_B =_read_and_compare_test_result(READ_STAT_B, 0x6AAA);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_SELF_TEST_STAT);
        return FAIL;
    }

    return SUCCESS;
}

ErrorFlag LTC6804_test_clear_all() {
    //STATST command (self test)
    bool cmp_A, cmp_B, cmp_C, cmp_D;

    LTC6804_send_start_command(CLEAR_REG_CV);
    DEVICE_DELAY_US(100);
    cmp_A =_read_and_compare_test_result(READ_CV_A, 0xFFFF);
    cmp_B =_read_and_compare_test_result(READ_CV_B, 0xFFFF);
    cmp_C =_read_and_compare_test_result(READ_CV_C, 0xFFFF);
    cmp_D =_read_and_compare_test_result(READ_CV_D, 0xFFFF);
    if (cmp_A || cmp_B || cmp_C || cmp_D){
        set_error_bit(ERROR_SLAVE_CLEAR_TEST);
        return FAIL;
    }

    LTC6804_send_start_command(CLEAR_REG_AUX);
    DEVICE_DELAY_US(100);
    cmp_A =_read_and_compare_test_result(READ_AUX_A, 0xFFFF);
    cmp_B =_read_and_compare_test_result(READ_AUX_B, 0xFFFF);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_CLEAR_TEST);
        return FAIL;
    }

    LTC6804_send_start_command(CLEAR_REG_STAT);
    DEVICE_DELAY_US(100);
    cmp_A =_read_and_compare_test_result(READ_STAT_A, 0xFFFF);
    // Compare to 0xFFFF but do not compare REV and RSVD
    // cmp_B =_read_and_compare_test_result(READ_STAT_B, 0xFFFF);
    if (cmp_A || cmp_B){
        set_error_bit(ERROR_SLAVE_CLEAR_TEST);
        return FAIL;
    }

    return SUCCESS;
}


ErrorFlag LTC6804_test_internal_diagnostic() {
    // DIAGN
    // internal mux test
    // MUXFAIL bit is also set to 1 on power-up (POR) or after a CLRSTAT command
    LTC6804_send_start_command(START_DIAGNOSE_MUX); // DIAGN
    DEVICE_DELAY_US(4500);
    slave_read_data(READ_STAT_B);
    for (uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        if (g_BatteryPack.slave[i].muxfail){
            set_error_bit(ERROR_SLAVE_MUXFAIL);
            return FAIL;
        }
    }

    return SUCCESS;
}


ErrorFlag LTC6804_test_accuracy_with_vref2() {
    // Accuracy Check
    // Measuring an independent voltage reference is the best means to verify the accuracy of a data acquisition system.
    // The LTC6804 contains a 2nd reference for this purpose.
    // The ADAX command will initiate the measurement of the 2nd reference.
    // The results are placed in auxiliary register group B.
    // The range of the result depends on the ADC measurement accuracy and the accuracy of the 2nd reference, including thermal hysteresis and long term drift.
    // Readings outside the range 2.985 to 3.015 indicate the system is out of its specified tolerance.

    uint16_t cmd;
    cmd = format_command_ADAX(ADC_MODE_NORMAL, AUX_CH_VREF2);
    LTC6804_send_start_command(cmd);
    DEVICE_DELAY_US(4500);

    slave_read_data(READ_AUX_B);

    for(uint16_t i = 0; i < TOTAL_SLAVES; i++) {
        uint16_t vref2 = g_BatteryPack.slave[i].vref2;
        if (vref2 > 30150 || vref2 < 29850){
            set_error_bit(ERROR_SLAVE_ACCURACY);
            return FAIL;
        }
    }

    return SUCCESS;
}


