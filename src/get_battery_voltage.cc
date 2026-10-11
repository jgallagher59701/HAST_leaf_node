//
// Read the battery voltage for the Leaf node using a 1M | 1M voltage
// divider and the RocketScream/M0 ADC.
//
// James Gallagher <jgallagher@opendap.org>
// 9/26/21

#include <Arduino.h>

#include "get_battery_voltage.h"
#include "leaf_node_logic.h"

#define V_BAT A1 // A5

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096
#define VALUES_TO_AVG 10
#define VOLTAGE_OFFSET 0.01

/**
 * @brief Read the current voltage of the battery 
 * 
 * @note this version assumes that a voltage divider reduces Vbat by 1/2
 * @return The battery voltage x 100 as an int
 */
int get_battery_voltage() {
    // Take one sample, discard it, then 10 samples
    // and return their average. Time delays, etc., can be set in
    // setup() using ADC calls. See:
    // https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/

    (void)analogRead(V_BAT); // discard the first read

    long raw = 0;
    for (int i = 0; i < VALUES_TO_AVG; ++i) {
        raw += analogRead(V_BAT);
    }

    return adc_average_to_centivolts(raw, VALUES_TO_AVG);
}

void get_battery_voltage_setup() {
    analogReference(AR_INTERNAL2V23);
    // See https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
    // And assume an input impedance of 500k for the 1M | 1M V divider
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(2);

    analogReadResolution(ADC_BITS);
}
