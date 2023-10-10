
#include <unity.h>

#include "standby_time.h"

// FIXME Rewrite these for the standby_time functions.


// TEST_ASSERT_EQUAL(join_request, get_message_type((void*)&jr));
// TEST_ASSERT_TRUE(parse_join_request(&jr, &dev_eui)); 
// TEST_ASSERT_TRUE_MESSAGE(strlen(str1) < 64, str1);

void test_compute_samples_input() {
    TEST_ASSERT_FALSE(compute_samples(0));
    TEST_ASSERT_FALSE(compute_samples(61));
    TEST_ASSERT_FALSE(compute_samples(17));

    TEST_ASSERT_TRUE(compute_samples(1));
    TEST_ASSERT_TRUE(compute_samples(5));
    TEST_ASSERT_TRUE(compute_samples(60));
}

int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_compute_samples_input);
    //RUN_TEST(test_parse_join_request);
    //RUN_TEST(test_join_request_to_string);

    UNITY_END();
}
