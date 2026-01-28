#include "Trace.h"
#include <zephyr/ztest.h>

ZTEST(Trace_test, test_get_value)
{
    Trace trace;
    zassert_trufalse(trace.sample(0).has_value());
}

ZTEST_SUITE(Trace_test, NULL, NULL, NULL, NULL, NULL);
