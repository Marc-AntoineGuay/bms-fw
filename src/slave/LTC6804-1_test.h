/*
 * LTC6804_tests.h
 *
 *  Created on: Mar 31, 2018
 *      Author: louis
 */

#ifndef SRC_SLAVE_LTC6804_TESTS_H_
#define SRC_SLAVE_LTC6804_TESTS_H_

#include "LTC6804-1.h"
#include "slave.h"

ErrorFlag LTC6804_test_cell_voltage();
ErrorFlag LTC6804_test_GPIO();
ErrorFlag LTC6804_test_status_register();
ErrorFlag LTC6804_test_internal_diagnostic();
ErrorFlag LTC6804_test_open_wire();
ErrorFlag LTC6804_test_accuracy_with_vref2();
ErrorFlag LTC6804_test_clear_all();

bool _read_and_compare_test_result(uint16_t read_command, uint16_t cmp);
bool _compare_read_data(LTC6804_Group *data, uint16_t cmp);

#endif /* SRC_SLAVE_LTC6804_TESTS_H_ */
