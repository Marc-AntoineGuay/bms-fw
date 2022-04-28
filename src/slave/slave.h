/*
 * slave.h
 *
 *  Created on: Mar 31, 2018
 *      Author: Louis-Philippe Asselin
 *
 *  Higher level interface for LTC6804-1 slave devices
 *
 */

#ifndef SRC_SLAVE_SLAVE_H_
#define SRC_SLAVE_SLAVE_H_

#include <src/defines.h>
#include <src/slave/LTC6804-1.h>
#include <src/slave/LTC6804-1_test.h>

void slave_read_CV_and_temperature();
void slave_read_all_CV();
void slave_read_temperature_sensor();
void slave_start_next_temperature_sensor();
void slave_read_config();
void slave_initialize();
void slave_test();
void slave_write_current_config();
void slave_update_cell_discharge_config(uint16_t slave_number, uint16_t discharge_config);
ErrorFlag slave_read_data(LTC6804_ReadCommand read_command);

ErrorFlag slave_check_and_update_bms_data(LTC6804_ReadCommand read_command, LTC6804_Group *data);



#endif /* SRC_SLAVE_SLAVE_H_ */
