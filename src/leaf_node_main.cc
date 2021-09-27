//
// Test get_battery_voltage()
//
// James Gallagher 9/26/21

#include <Arduino.h>

#include "get_bat_voltage.h"

#define Serial SerialUSB // Needed for RS. jhrg 7/26/20

void setup() {

    get_battery_voltage_setup();
#if 1

    Serial.begin(115200);
    while (!Serial)
        ;


    Serial.println("Begin...");
#endif
}

void loop() {
#if 1
    Serial.print("get_battery_voltage(): ");
    Serial.println(get_battery_voltage());
#endif
    get_battery_voltage();
    delay(1000);
}