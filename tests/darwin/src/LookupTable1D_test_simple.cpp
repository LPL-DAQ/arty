#include "LookupTable1D.h"
#include "LookupTable1D_test_lut_simple.h"
#include <zephyr/ztest.h>

ZTEST(Trace_test_simple, test_left_boundary)
{
    Trace trace;
    zassert_false(trace.sample(0).has_value());
}

ZTEST_SUITE(LookupTable1D_test_simple, NULL, NULL, NULL, NULL, NULL);
