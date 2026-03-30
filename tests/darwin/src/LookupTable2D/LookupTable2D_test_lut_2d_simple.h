/*
>>> GENERATED FILE <<<

Re-create this whenever tests/darwin/src/LookupTable2D/LookupTable2D_test_simple.csv changes by running from the arty directory:

```
uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_2d.py test_lut_2d_simple 2 -1.0 1.0 2.0 3 0.0 6.0 3.0 tests/darwin/src/LookupTable2D/LookupTable2D_test_simple.csv tests/darwin/src/LookupTable2D/LookupTable2D_test_lut_2d_simple.h
```
*/

#pragma once

#include <array>
#include "LookupTable2D.h"

constexpr int TEST_LUT_2D_SIMPLE_X_LEN = 2;
constexpr float TEST_LUT_2D_SIMPLE_X_MIN = -1.0000000000f;
constexpr float TEST_LUT_2D_SIMPLE_X_MAX = 1.0000000000f;
constexpr float TEST_LUT_2D_SIMPLE_X_GAP = 2.0000000000f;

constexpr int TEST_LUT_2D_SIMPLE_Y_LEN = 3;
constexpr float TEST_LUT_2D_SIMPLE_Y_MIN = 0.0000000000f;
constexpr float TEST_LUT_2D_SIMPLE_Y_MAX = 6.0000000000f;
constexpr float TEST_LUT_2D_SIMPLE_Y_GAP = 3.0000000000f;

constexpr std::array<std::array<float, TEST_LUT_2D_SIMPLE_Y_LEN>, TEST_LUT_2D_SIMPLE_X_LEN> TEST_LUT_2D_SIMPLE_BPS {std::array<float, TEST_LUT_2D_SIMPLE_Y_LEN>{-10.0000000000f, 0.0000000000f, 10.0000000000f}, std::array<float, TEST_LUT_2D_SIMPLE_Y_LEN>{0.0000000000f, 10.0000000000f, 20.0000000000f}};

typedef LookupTable2D<TEST_LUT_2D_SIMPLE_X_LEN, TEST_LUT_2D_SIMPLE_X_MIN, TEST_LUT_2D_SIMPLE_X_MAX, TEST_LUT_2D_SIMPLE_X_GAP, TEST_LUT_2D_SIMPLE_Y_LEN, TEST_LUT_2D_SIMPLE_Y_MIN, TEST_LUT_2D_SIMPLE_Y_MAX, TEST_LUT_2D_SIMPLE_Y_GAP, TEST_LUT_2D_SIMPLE_BPS> TestLut2dSimple;
