//
// Native tests for the Arduino-free logic layer extracted out of leaf_node.cc /
// get_battery_voltage.cc (TASK-001, IC-001).
// Run with: pio test -e native
//

#include <stdlib.h>

#include <unity.h>

#include "leaf_node_logic.h"

void setUp() {
    // get_epoch() uses mktime(), which is local-time-dependent; fix the host's
    // zone so the expected epoch values below don't depend on where this runs.
    setenv("TZ", "UTC", 1);
    tzset();
}

void tearDown() {}

// Normal case: a two-digit-day date, matching __DATE__'s "Mmm dd yyyy" format.
void test_get_epoch_known_date() {
    TEST_ASSERT_EQUAL_INT64(1790605800, (int64_t)get_epoch("Sep 28 2026", "14:30:00"));
}

// Boundary: the unix epoch itself.
void test_get_epoch_unix_epoch() {
    TEST_ASSERT_EQUAL_INT64(0, (int64_t)get_epoch("Jan  1 1970", "00:00:00"));
}

// Normal case plus the period boundary (message_count == period, and a multiple).
void test_should_request_time_period_boundaries() {
    TEST_ASSERT_FALSE(should_request_time(1, 24));
    TEST_ASSERT_FALSE(should_request_time(23, 24));
    TEST_ASSERT_TRUE(should_request_time(24, 24));
    TEST_ASSERT_TRUE(should_request_time(48, 24));
    TEST_ASSERT_TRUE(should_request_time(0, 24));  // message counter starts at 0 before the pre-increment
}

// Clearly-invalid input: period == 0 would divide by zero if not guarded.
void test_should_request_time_zero_period() {
    TEST_ASSERT_FALSE(should_request_time(0, 0));
    TEST_ASSERT_FALSE(should_request_time(24, 0));
}

void test_clear_transient_status_preserves_high_nibble() {
    TEST_ASSERT_EQUAL_HEX8(0xF0, clear_transient_status(0xFF));
    TEST_ASSERT_EQUAL_HEX8(0x00, clear_transient_status(0x0F));
    TEST_ASSERT_EQUAL_HEX8(0xA0, clear_transient_status(0xAF));
}

// Normal case: ones digit increments.
void test_bump_log_file_name_increments_ones_digit() {
    char name[13] = "Data00.csv";
    TEST_ASSERT_TRUE(bump_log_file_name(name, 4));
    TEST_ASSERT_EQUAL_STRING("Data01.csv", name);
}

// Carry: ones digit wraps 9->0 and the tens digit increments.
void test_bump_log_file_name_carries_tens_digit() {
    char name[13] = "Data09.csv";
    TEST_ASSERT_TRUE(bump_log_file_name(name, 4));
    TEST_ASSERT_EQUAL_STRING("Data10.csv", name);
}

// Boundary: 99 is exhausted - false, and name is left unchanged (the
// adapter's exhausted-name fallback relies on this to keep reusing "99").
void test_bump_log_file_name_exhausted_at_99() {
    char name[13] = "Data99.csv";
    TEST_ASSERT_FALSE(bump_log_file_name(name, 4));
    TEST_ASSERT_EQUAL_STRING("Data99.csv", name);
}

// Normal case: a large delta should resync.
void test_should_resync_time_large_delta() {
    int32_t delta = 0;
    TEST_ASSERT_TRUE(should_resync_time(1000, 990, &delta));
    TEST_ASSERT_EQUAL_INT32(10, delta);

    TEST_ASSERT_TRUE(should_resync_time(990, 1000, &delta));
    TEST_ASSERT_EQUAL_INT32(-10, delta);
}

// Boundary: exactly a 1-second delta (either direction) must NOT resync - only
// abs(delta) > 1 does.
void test_should_resync_time_one_second_boundary() {
    int32_t delta = 99;
    TEST_ASSERT_FALSE(should_resync_time(1001, 1000, &delta));
    TEST_ASSERT_EQUAL_INT32(1, delta);

    TEST_ASSERT_FALSE(should_resync_time(999, 1000, &delta));
    TEST_ASSERT_EQUAL_INT32(-1, delta);

    TEST_ASSERT_FALSE(should_resync_time(1000, 1000, &delta));
    TEST_ASSERT_EQUAL_INT32(0, delta);
}

// Clearly-invalid input: a null out_delta must not crash.
void test_should_resync_time_null_out_delta() {
    TEST_ASSERT_TRUE(should_resync_time(1000, 990, nullptr));
    TEST_ASSERT_FALSE(should_resync_time(1000, 1000, nullptr));
}

// Normal case: full-scale ADC reading (raw == 4096 * n_samples) should read
// close to the ADC's own top-of-range voltage once the formula is applied.
void test_adc_average_to_centivolts_full_scale() {
    // avg = 4096 -> voltage = 4.46 * (4096/4096) = 4.46 -> + 0.01 offset -> 447
    TEST_ASSERT_EQUAL_INT(447, adc_average_to_centivolts(4096L * 10, 10));
}

// Normal case: zero reading still applies the fixed offset.
void test_adc_average_to_centivolts_zero_reading() {
    // avg = 0 -> voltage = 0 -> + 0.01 offset -> 1
    TEST_ASSERT_EQUAL_INT(1, adc_average_to_centivolts(0, 10));
}

// Clearly-invalid input: n_samples == 0 would divide by zero if not guarded.
void test_adc_average_to_centivolts_zero_samples() {
    TEST_ASSERT_EQUAL_INT(0, adc_average_to_centivolts(1000, 0));
    TEST_ASSERT_EQUAL_INT(0, adc_average_to_centivolts(0, -1));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_get_epoch_known_date);
    RUN_TEST(test_get_epoch_unix_epoch);
    RUN_TEST(test_should_request_time_period_boundaries);
    RUN_TEST(test_should_request_time_zero_period);
    RUN_TEST(test_clear_transient_status_preserves_high_nibble);
    RUN_TEST(test_bump_log_file_name_increments_ones_digit);
    RUN_TEST(test_bump_log_file_name_carries_tens_digit);
    RUN_TEST(test_bump_log_file_name_exhausted_at_99);
    RUN_TEST(test_should_resync_time_large_delta);
    RUN_TEST(test_should_resync_time_one_second_boundary);
    RUN_TEST(test_should_resync_time_null_out_delta);
    RUN_TEST(test_adc_average_to_centivolts_full_scale);
    RUN_TEST(test_adc_average_to_centivolts_zero_reading);
    RUN_TEST(test_adc_average_to_centivolts_zero_samples);
    return UNITY_END();
}
