//
// Read the battery voltage for the Leaf node using a 1M | 1M voltage
// divider and the RoscketScream/M0 ADC.
//
// James Gallagher <jgallagher@opendap.org>
// 9/26/21

#ifndef get_bat_voltage_h
#define get_bat_voltage_h

int get_battery_voltage();
void get_battery_voltage_setup();

#endif
