#include "LookupTable1D.h"
#include "LookupTable1D_test_lut_simple.h"
#include <iostream>
#include <zephyr/ztest.h>

constexpr float EPSILON = 0.001f;

ZTEST(LookupTable1D_test_simple, test_left_boundary)
{
    float val = TestLutSimple::sample(-5.0f);
    // std::cout << "Got: " << val << std::endl;
    zassert_within(val, -50.0f, EPSILON);
}

ZTEST(LookupTable1D_test_simple, test_right_boundary)
{
    float val = TestLutSimple::sample(5.0f);
    // std::cout << "Got: " << val << std::endl;
    zassert_within(val, 50.0f, EPSILON, "val=%f");
}

ZTEST(LookupTable1D_test_simple, test_intermediate_values)
{
    for (int i = 1; i < 1000; ++i) {
        float in = -5.0f + i * 0.01f;
        float expected_out = in * 10.0f;
        float actual_out = TestLutSimple::sample(in);

        // std::cout << "Got: in=" << in << ", expected_out=" << expected_out << ", actual_out=" << actual_out << std::endl;
        zassert_within(expected_out, actual_out, EPSILON);
    }
}

ZTEST(LookupTable1D_test_simple, test_clamp_left)
{
    for (int i = 1; i < 1000; ++i) {
        float in = -5.0f - i;
        float val = TestLutSimple::sample(in);

        // std::cout << "in=" << in << ", val=" << val << std::endl;
        zassert_within(val, -50.0f, EPSILON);
    }
}

ZTEST(LookupTable1D_test_simple, test_clamp_right)
{
    for (int i = 1; i < 1000; ++i) {
        float in = 5.0f + i;
        float val = TestLutSimple::sample(in);

        // std::cout << "in=" << in << ", val=" << val << std::endl;
        zassert_within(val, 50.0f, EPSILON);
    }
}

ZTEST_SUITE(LookupTable1D_test_simple, NULL, NULL, NULL, NULL, NULL);
