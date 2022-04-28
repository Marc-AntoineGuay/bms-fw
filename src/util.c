/*
 * util.c
 *
 *  Created on: Feb 7, 2018
 *      Author: Louis-Philippe Asselin
 */

#include "util.h"

  ////////////////
 /// Helpers ///
//////////////

  /////////////////////
 /// Implementaton ///
/////////////////////

int16_t convertThermistanceToCelsius(uint16_t adc_value){
    if (adc_value <1520) return INT16_MAX; // not in table
    else if (adc_value >25449) return INT16_MAX; // not in table
    else {
        uint16_t index;
        index = (adc_value - 1520) / 8;
        return THERMISTOR_LOOKUP_TABLE[index];
    }

}

uint16_t convertCelsiusToThermistance(uint16_t temperature_celsius){
    float float_celsius;
    float adc_unit;
    uint16_t uint_adc_unit;

    float_celsius = (int32_t) temperature_celsius;
    adc_unit = (float_celsius - 94.049611) *10000 / -48.560832;
    // Check value. test with input 21. should return something around 15000
    uint_adc_unit = (uint16_t) adc_unit;

    return uint_adc_unit;
}


uint16_t convertPackChargeToCoulombs(int64_t coulomb_counter){
    int64_t coulombs;
    coulombs = coulomb_counter * 450 /1862 /CURRENT_PROBE_SAMPLING_FREQ;
    return (uint16_t) coulombs;
}

uint16_t convertCvForSending(uint16_t cv_value) {
    return (uint16_t)(cv_value*0.005);
}

void splitUint16InUint8(uint16_t value, uint16_t *high, uint16_t *low) {
    *high = value >> 8;
    *low = value & 0xFF;
}

uint16_t convertMasterThermistanceToCelcius(uint16_t adc_value) {
    uint16_t retval = INT16_MAX;

    if (adc_value < 208 || adc_value > 2526) { // out of bounds
        return INT16_MAX;
    }

    float pow2 = (float)adc_value*(float)adc_value;
    float pow3 = pow2*(float)adc_value;

    retval = (uint16_t)(pow3*-1.2e-8 + pow2*6e-5 - adc_value*0.12 + 120);

    return retval;
}
