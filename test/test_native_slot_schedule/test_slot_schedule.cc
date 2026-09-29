//
// Native tests for the node-address -> transmit slot mapping (FR-011, FR-012, IC-005).
// Run with: pio test -e native
//

#include <unity.h>

#include "slot_schedule.h"

void setUp() {}
void tearDown() {}

static uint16_t start_of(uint8_t node, SlotSchedule s, uint16_t grouped_start_s) {
    uint16_t start = 0xFFFF;
    TEST_ASSERT_TRUE(slot_start_s(node, s, grouped_start_s, &start));
    return start;
}

// FR-011: node N starts at minute N-1, second 0.
void test_staggered_layout() {
    TEST_ASSERT_EQUAL_UINT16(0, start_of(1, SlotSchedule::Staggered, 0));
    TEST_ASSERT_EQUAL_UINT16(11 * 60, start_of(12, SlotSchedule::Staggered, 0));
    TEST_ASSERT_EQUAL_UINT16(12 * 60, start_of(13, SlotSchedule::Staggered, 0));
    TEST_ASSERT_EQUAL_UINT16(14 * 60, start_of(15, SlotSchedule::Staggered, 0));
}

// The grouped start time is ignored by the staggered layout.
void test_staggered_ignores_grouped_start() {
    TEST_ASSERT_EQUAL_UINT16(4 * 60, start_of(5, SlotSchedule::Staggered, 1234));
}

// FR-012: node N starts 5 * (N-1) s after the common start; 15 slots span 75 s.
void test_grouped_layout() {
    TEST_ASSERT_EQUAL_UINT16(0, start_of(1, SlotSchedule::Grouped, 0));
    TEST_ASSERT_EQUAL_UINT16(55, start_of(12, SlotSchedule::Grouped, 0));
    TEST_ASSERT_EQUAL_UINT16(60, start_of(13, SlotSchedule::Grouped, 0));
    TEST_ASSERT_EQUAL_UINT16(70, start_of(15, SlotSchedule::Grouped, 0));
    TEST_ASSERT_EQUAL_UINT16(600 + 70, start_of(15, SlotSchedule::Grouped, 600));
}

// IC-005: only addresses 1..15 get a slot; 0 is the main node.
void test_rejects_out_of_range_nodes() {
    uint16_t start = 42;
    TEST_ASSERT_FALSE(slot_start_s(0, SlotSchedule::Staggered, 0, &start));
    TEST_ASSERT_FALSE(slot_start_s(16, SlotSchedule::Staggered, 0, &start));
    TEST_ASSERT_FALSE(slot_start_s(0, SlotSchedule::Grouped, 0, &start));
    TEST_ASSERT_FALSE(slot_start_s(16, SlotSchedule::Grouped, 0, &start));
    TEST_ASSERT_FALSE(slot_start_s(255, SlotSchedule::Grouped, 0, &start));
    TEST_ASSERT_EQUAL_UINT16(42, start);  // unchanged on failure
}

// A grouped window whose last slot would run past :59:59 is rejected.
void test_rejects_grouped_window_past_end_of_hour() {
    uint16_t start = 0;
    // Node 15's slot is [3525 + 70, 3600): ends exactly at the hour - allowed.
    TEST_ASSERT_TRUE(slot_start_s(15, SlotSchedule::Grouped, 3525, &start));
    TEST_ASSERT_EQUAL_UINT16(3595, start);
    // One second later it would end at 3601.
    TEST_ASSERT_FALSE(slot_start_s(15, SlotSchedule::Grouped, 3526, &start));
    // Very large start values must not wrap around into a valid slot.
    TEST_ASSERT_FALSE(slot_start_s(1, SlotSchedule::Grouped, 0xFFFF, &start));
}

void test_rejects_unknown_schedule_and_null_output() {
    uint16_t start = 0;
    TEST_ASSERT_FALSE(slot_start_s(1, static_cast<SlotSchedule>(7), 0, &start));
    TEST_ASSERT_FALSE(slot_start_s(1, SlotSchedule::Staggered, 0, nullptr));
}

// Compile-time form used by the static_assert in leaf_node.cc.
void test_slot_is_valid_is_constexpr() {
    static_assert(slot_is_valid(1, SlotSchedule::Staggered, 0), "node 1 staggered");
    static_assert(slot_is_valid(15, SlotSchedule::Grouped, 0), "node 15 grouped");
    static_assert(!slot_is_valid(0, SlotSchedule::Staggered, 0), "node 0 is the main node");
    static_assert(!slot_is_valid(16, SlotSchedule::Staggered, 0), "IC-005 limit");
    TEST_PASS();
}

void test_slot_to_mmss() {
    uint8_t mm = 99, ss = 99;
    TEST_ASSERT_TRUE(slot_to_mmss(0, &mm, &ss));
    TEST_ASSERT_EQUAL_UINT8(0, mm);
    TEST_ASSERT_EQUAL_UINT8(0, ss);

    TEST_ASSERT_TRUE(slot_to_mmss(14 * 60, &mm, &ss));
    TEST_ASSERT_EQUAL_UINT8(14, mm);
    TEST_ASSERT_EQUAL_UINT8(0, ss);

    TEST_ASSERT_TRUE(slot_to_mmss(70, &mm, &ss));
    TEST_ASSERT_EQUAL_UINT8(1, mm);
    TEST_ASSERT_EQUAL_UINT8(10, ss);

    TEST_ASSERT_TRUE(slot_to_mmss(3599, &mm, &ss));
    TEST_ASSERT_EQUAL_UINT8(59, mm);
    TEST_ASSERT_EQUAL_UINT8(59, ss);
}

void test_slot_to_mmss_rejects_bad_input() {
    uint8_t mm = 7, ss = 8;
    TEST_ASSERT_FALSE(slot_to_mmss(3600, &mm, &ss));
    TEST_ASSERT_EQUAL_UINT8(7, mm);  // unchanged on failure
    TEST_ASSERT_EQUAL_UINT8(8, ss);
    TEST_ASSERT_FALSE(slot_to_mmss(0, nullptr, &ss));
    TEST_ASSERT_FALSE(slot_to_mmss(0, &mm, nullptr));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_staggered_layout);
    RUN_TEST(test_staggered_ignores_grouped_start);
    RUN_TEST(test_grouped_layout);
    RUN_TEST(test_rejects_out_of_range_nodes);
    RUN_TEST(test_rejects_grouped_window_past_end_of_hour);
    RUN_TEST(test_rejects_unknown_schedule_and_null_output);
    RUN_TEST(test_slot_is_valid_is_constexpr);
    RUN_TEST(test_slot_to_mmss);
    RUN_TEST(test_slot_to_mmss_rejects_bad_input);
    return UNITY_END();
}
