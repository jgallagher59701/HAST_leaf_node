//
// Test get_battery_voltage()
//
// James Gallagher 9/26/21

#include <Arduino.h>

#include "get_bat_voltage.h"

#define Serial SerialUSB // Needed for RS. jhrg 7/26/20

void setup() {

    get_battery_voltage_setup();

    Serial.begin(115200);
#if 0
    while (!Serial)
        ;
#endif

    Serial.println("Begin...");
}

void loop() {
    Serial.print("get_battery_voltage(): ");
    Serial.println(get_battery_voltage());
    delay(1000);
}