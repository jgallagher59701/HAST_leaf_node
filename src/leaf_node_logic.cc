//
// Arduino-free logic layer extracted out of leaf_node.cc / get_battery_voltage.cc
// (TASK-001, IC-001).
//
// James Gallagher <jgallagher@opendap.org>
// 10/11/26

#include "leaf_node_logic.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

time_t get_epoch(const char *date, const char *time) {
    char s_month[5];
    struct tm t = {}; // zero-initialize every field (avoids -Wmissing-field-initializers)
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

    sscanf(date, "%s %d %d", s_month, &t.tm_mday, &t.tm_year);

    // pointer math
    int month = (strstr(month_names, s_month) - month_names) / 3;

    t.tm_mon = month;
    t.tm_year -= 1900;
    t.tm_isdst = -1;

    sscanf(time, "%d:%d:%d", &t.tm_hour, &t.tm_min, &t.tm_sec);

    return mktime(&t);
}

bool should_request_time(uint32_t message_count, uint32_t period) {
    return period != 0 && message_count % period == 0;
}

uint8_t clear_transient_status(uint8_t status) {
    return status & 0xF0;
}

bool should_resync_time(uint32_t main_node_time, uint32_t local_time, int32_t *out_delta) {
    int32_t delta_time = (int32_t)(main_node_time - local_time);
    if (out_delta)
        *out_delta = delta_time;

    int32_t magnitude = delta_time < 0 ? -delta_time : delta_time;
    return magnitude > 1;
}

int adc_average_to_centivolts(long raw_sum, int n_samples) {
    if (n_samples <= 0)
        return 0;

    // Matches get_battery_voltage.cc's ADC_MAX_VALUE (12-bit ADC) and
    // VOLTAGE_OFFSET (measured calibration bias) constants exactly.
    constexpr float ADC_MAX_VALUE = 4096.0f;
    constexpr float VOLTAGE_OFFSET = 0.01f;

    float avg_adc_reading = (float)raw_sum / n_samples;

    // Vref = 2.23, the voltage divider cuts the voltage in half. ADC/ADC_MAX *
    // 2.23 is the actual voltage the ADC measures. Double that to account for
    // the voltage divider. The divider keeps the voltage measured below 3.3
    // (Vbat can be 3.7V).
    float voltage = 4.46f * (avg_adc_reading / ADC_MAX_VALUE);

    return (int)roundf((voltage + VOLTAGE_OFFSET) * 100.0f); // voltage * 100
}

bool bump_log_file_name(char *name, uint8_t base_name_size) {
    if (name[base_name_size + 1] != '9') {
        name[base_name_size + 1]++;
        return true;
    } else if (name[base_name_size] != '9') {
        name[base_name_size + 1] = '0';
        name[base_name_size]++;
        return true;
    }
    return false;
}
