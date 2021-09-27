//
// Test get_battery_voltage()
//
// James Gallagher 9/26/21

#include <Arduino.h>

#include "get_bat_voltage.h"

void setup() {
    Serial.begin(115200);

    while (!Serial)
        ;

    get_battery_voltage_setup();

    Serial.println("Begin...");
}

void loop() {
    Serial.print("get_battery_voltage(): ");
    Serial.println(get_battery_voltage());
    delay(1000);
}