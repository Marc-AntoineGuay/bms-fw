/*
 * util.h
 *
 *  Created on: Feb 7, 2018
 *      Author: Louis-Philippe Asselin
 */

#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_

#include <device.h>
#include "defines.h"
#include "thermistor_lookup.h"

/*!
 * Converts ADC sampled thermistance value to Celsius
 * returns an integer. 1 = 1degC
 *
 * Uses a lookup table
 * https://docs.google.com/spreadsheets/d/1sNG3cwA7LgruL5ftnRLfCp38skVUsABsJqtSGXZg6QY/edit?usp=sharing
 */
int16_t convertThermistanceToCelsius(uint16_t adc_value);

/**
 * Opposite of convert_thermistance_to_celsius(adc_value)
 *
 * Not super accurate
 *
 * XXX Precompute and remove this function
 */
uint16_t convertCelsiusToThermistance(uint16_t temperature_celsius);

/**
 * Current probe adc units to coulombs
 */
uint16_t convertPackChargeToCoulombs(int64_t coulomb_counter);

/**
 * Converts cell voltage value as returned by LTC's to a message format
 *
 * Returns the cv value that fit in one byte. In order to rebuild the cell voltage,
 * multiply by two then divide by 100
 */
uint16_t convertCvForSending(uint16_t cv_value);

uint16_t convertMasterThermistanceToCelcius(uint16_t adc_value);

void splitUint16InUint8(uint16_t value, uint16_t *high, uint16_t *low);

#endif /* SRC_UTIL_H_ */
