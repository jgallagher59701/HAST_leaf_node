//
// Native tests for the join_request/join_response messages in
// lib/soil_sensor_common/messages.cc (BUG-002: parse_join_response checked the
// wrong MessageType). Only reachable with SUPPORT_JOIN defined - see
// platformio.ini's [env:native] build_flags.
// Run with: pio test -e native
//

#include <unity.h>

#include "messages.h"

void setUp() {}
void tearDown() {}

// Normal case: a correctly-typed join_response_t round-trips.
void test_parse_join_response_accepts_join_response() {
    join_response_t jr;
    build_join_response(&jr, 5, 1728648621);

    uint8_t node = 0;
    uint32_t time = 0;
    TEST_ASSERT_TRUE(parse_join_response(&jr, &node, &time));
    TEST_ASSERT_EQUAL_UINT8(5, node);
    TEST_ASSERT_EQUAL_UINT32(1728648621, time);
}

// The bug (BUG-002): a join_request_t-typed buffer must be rejected, not
// misparsed as a join_response_t. Before the fix, parse_join_response() checked
// data->type != join_request - the exact opposite of what it should reject.
void test_parse_join_response_rejects_join_request() {
    join_request_t req;
    build_join_request(&req, 0x0123456789ABCDEFULL);

    uint8_t node = 99;
    uint32_t time = 99;
    TEST_ASSERT_FALSE(parse_join_response(reinterpret_cast<const join_response_t *>(&req), &node, &time));
    TEST_ASSERT_EQUAL_UINT8(99, node);  // unchanged on failure
    TEST_ASSERT_EQUAL_UINT32(99, time);
}

// Null out-params are optional (both parse_join_response and parse_join_request
// treat them as "don't care").
void test_parse_join_response_null_out_params() {
    join_response_t jr;
    build_join_response(&jr, 5, 1728648621);
    TEST_ASSERT_TRUE(parse_join_response(&jr, nullptr, nullptr));
}

// Sibling function, for contrast: parse_join_request already checked the right
// type before this bug existed - confirms the test harness/SUPPORT_JOIN wiring
// itself works independently of the bug under test.
void test_parse_join_request_round_trip() {
    join_request_t req;
    build_join_request(&req, 0x0123456789ABCDEFULL);

    uint64_t dev_eui = 0;
    TEST_ASSERT_TRUE(parse_join_request(&req, &dev_eui));
    TEST_ASSERT_EQUAL_UINT64(0x0123456789ABCDEFULL, dev_eui);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parse_join_response_accepts_join_response);
    RUN_TEST(test_parse_join_response_rejects_join_request);
    RUN_TEST(test_parse_join_response_null_out_params);
    RUN_TEST(test_parse_join_request_round_trip);
    return UNITY_END();
}
