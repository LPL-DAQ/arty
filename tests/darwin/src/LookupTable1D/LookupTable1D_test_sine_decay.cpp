#include "LookupTable1D.h"
#include "LookupTable1D_test_lut_sine_decay.h"
#include <cmath>
#include <iostream>
#include <zephyr/ztest.h>

constexpr float EPSILON = 0.001f;

ZTEST(LookupTable1D_test_sine_decay, test_main_domain)
{
    float x = -1.0f;
    while (x <= 5.0f) {
        float expected_out = sinf(20.0f * x) / (x + 1.1f);
        float actual_out = TestLutSineDecay::sample(x);
        std::cout << "x=" << x << ", expected_out=" << expected_out << ", actual_out=" << actual_out << std::endl;
        zassert_within(expected_out, actual_out, EPSILON);

        x += 0.0015f;
    }
}

ZTEST(LookupTable1D_test_sine_decay, test_clamp_left)
{
    for (int i = 1; i < 1000; ++i) {
        float in = -1.0f - i / 0.01f;
        float val = TestLutSineDecay::sample(in);

        std::cout << "in=" << in << ", val=" << val << std::endl;
        zassert_within(val, -9.129452507276268f, EPSILON);
    }
}

ZTEST(LookupTable1D_test_sine_decay, test_clamp_right)
{
    for (int i = 1; i < 1000; ++i) {
        float in = 5.0f + i / 0.01f;
        float val = TestLutSineDecay::sample(in);

        std::cout << "in=" << in << ", val=" << val << std::endl;
        zassert_within(val, -0.08301076083764923f, EPSILON);
    }
}

ZTEST_SUITE(LookupTable1D_test_sine_decay, NULL, NULL, NULL, NULL, NULL);
