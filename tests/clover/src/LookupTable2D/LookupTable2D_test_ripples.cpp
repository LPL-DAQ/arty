#include "LookupTable2D_test_lut_2d_ripples.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <zephyr/ztest.h>

constexpr float EPSILON = 0.001f;

// Stress test against ripple function lookup table

ZTEST(LookupTable2D_test_ripples, test_ripples)
{
    for (float x = -5.0f; x <= 6.0f; x += 0.17f) {
        for (float y = -6.0f; y <= 4.0f; y += 0.17f) {
            float x_clamped = std::clamp(x, -4.0f, 5.0f);
            float y_clamped = std::clamp(y, -5.0f, 3.0f);

            float expected_val = std::sin(std::sqrt(std::pow(x_clamped + 3.0f, 2.0f) + std::pow(y_clamped - 1.0f, 2.0f))) * (x_clamped + 0.2f * y_clamped);
            float actual_val = TestLut2dRipples::sample(x, y);
            std::cout << "x=" << x << ", y=" << y << ", expected_val=" << expected_val << ", actual_val=" << actual_val
                      << std::endl;
            zassert_within(expected_val, actual_val, EPSILON);
        }
    }
}

ZTEST_SUITE(LookupTable2D_test_ripples, NULL, NULL, NULL, NULL, NULL);
