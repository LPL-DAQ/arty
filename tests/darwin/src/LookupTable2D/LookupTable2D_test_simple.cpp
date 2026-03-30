#include "LookupTable2D_test_lut_2d_simple.h"
#include <iostream>
#include <zephyr/ztest.h>

constexpr float EPSILON = 0.001f;

// Middle points

ZTEST(LookupTable2D_test_simple, test_middle)
{
    float val = TestLut2dSimple::sample(0.0f, 1.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 0.0f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, 3.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 5.0f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, 4.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 10.0f, EPSILON);
}

// Edges

ZTEST(LookupTable2D_test_simple, test_x_low_edge)
{
    float val = TestLut2dSimple::sample(-1.0f, 1.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -5.0f, EPSILON);

    val = TestLut2dSimple::sample(-1.0f, 3.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 0.0f, EPSILON);

    val = TestLut2dSimple::sample(-1.0f, 4.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 5.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_edge)
{
    float val = TestLut2dSimple::sample(1.0f, 1.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 5.0f, EPSILON);

    val = TestLut2dSimple::sample(1.0f, 3.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 10.0f, EPSILON);

    val = TestLut2dSimple::sample(1.0f, 4.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 15.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_y_low_edge)
{
    float val = TestLut2dSimple::sample(-0.5f, 0.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -7.5f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, 0.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -5.0f, EPSILON);

    val = TestLut2dSimple::sample(0.5f, 0.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -2.5f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_y_high_edge)
{
    float val = TestLut2dSimple::sample(-0.5f, 6.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 12.5f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, 6.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 15.0f, EPSILON);

    val = TestLut2dSimple::sample(0.5f, 6.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 17.5f, EPSILON);
}

// Corners

ZTEST(LookupTable2D_test_simple, test_x_low_y_low_corner)
{
    float val = TestLut2dSimple::sample(-1.0f, 0.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -10.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_low_y_high_corner)
{
    float val = TestLut2dSimple::sample(-1.0f, 6.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 10.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_y_low_corner)
{
    float val = TestLut2dSimple::sample(1.0f, 0.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 0.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_y_high_corner)
{
    float val = TestLut2dSimple::sample(1.0f, 6.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 20.0f, EPSILON);
}

// Middle clamps

ZTEST(LookupTable2D_test_simple, test_x_low_y_middle_clamp)
{
    float val = TestLut2dSimple::sample(-1.5f, 1.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -5.0f, EPSILON);

    val = TestLut2dSimple::sample(-1.5f, 3.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 0.0f, EPSILON);

    val = TestLut2dSimple::sample(-1.5f, 4.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 5.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_y_middle_clamp)
{
    float val = TestLut2dSimple::sample(1.5f, 1.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 5.0f, EPSILON);

    val = TestLut2dSimple::sample(1.5f, 3.0f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 10.0f, EPSILON);

    val = TestLut2dSimple::sample(1.5f, 4.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 15.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_middle_y_low_clamp)
{
    float val = TestLut2dSimple::sample(-0.5f, -0.2f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -7.5f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, -0.2f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -5.0f, EPSILON);

    val = TestLut2dSimple::sample(0.5f, -0.2f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -2.5f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_middle_y_high_clamp)
{
    float val = TestLut2dSimple::sample(-0.5f, 6.005f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 12.5f, EPSILON);

    val = TestLut2dSimple::sample(0.0f, 6.005f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 15.0f, EPSILON);

    val = TestLut2dSimple::sample(0.5f, 6.005f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 17.5f, EPSILON);
}

// Corner clamps

ZTEST(LookupTable2D_test_simple, test_x_low_y_low_clamp)
{
    float val = TestLut2dSimple::sample(-1.5f, -0.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, -10.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_low_y_high_clamp)
{
    float val = TestLut2dSimple::sample(-1.5f, 6.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 10.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_y_low_clamp)
{
    float val = TestLut2dSimple::sample(1.5f, -0.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 0.0f, EPSILON);
}

ZTEST(LookupTable2D_test_simple, test_x_high_y_high_clamp)
{
    float val = TestLut2dSimple::sample(1.5f, 6.5f);
    std::cout << "Got: " << val << std::endl;
    zassert_within(val, 20.0f, EPSILON);
}

ZTEST_SUITE(LookupTable2D_test_simple, NULL, NULL, NULL, NULL, NULL);
