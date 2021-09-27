//
// Read the battery voltage for the Leaf node using a 1M | 1M voltage
// divider and the RoscketScream/M0 ADC.
//
// James Gallagher <jgallagher@opendap.org>
// 9/26/21

#include "get_bat_voltage.h"
#include <Arduino.h>

#define USE_AREF_2V23 1
#define V_BAT A1 // A5

#define ADC_BITS 12
#define ADC_MAX_VALUE 4096
#define VALUES_TO_AVG 10

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

    int raw = 0;
    for (int i = 0; i < VALUES_TO_AVG; ++i) {
        raw += analogRead(V_BAT);
    }

    float avg_adc_reading = (float)raw / VALUES_TO_AVG;

#if USE_AREF_2V23
    // Vref = 2.23, the voltage divider cuts the voltage in half. ADC /ADC_MAX
    // * 2.23 is the actual voltage the ADC measures. Double that to account
    // for the voltage divider. The divider is there to keep the voltage measured
    // below 3.3 (Vbat can be 3.7V).
    float voltage = 4.46 * (avg_adc_reading / ADC_MAX_VALUE);
#else
    // Using the 3V3 Vref fails because when Vbat falls below 3.3V, the value of
    // Vref tracks it, so all values of Vbat < 3.3 show up as 3.3V.
    float voltage = 4.3 * (avg_counts / (float)ADC_MAX_VALUE);
#endif
    return (int)roundf(voltage * 100.0); // voltage * 100
}

void get_battery_voltage_setup() {
#if USE_AREF_2V23
    analogReference(AR_INTERNAL2V23);
    // See https://blog.thea.codes/getting-the-most-out-of-the-samd21-adc/
    // And assume an input impedance of 500k for the 1M | 1M V divider
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(2);
#endif
    analogReadResolution(ADC_BITS);
}
