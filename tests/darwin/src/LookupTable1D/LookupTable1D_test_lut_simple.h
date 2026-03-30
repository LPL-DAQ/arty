#pragma once

/*
>>> GENERATED FILE <<<

Re-create this whenever tests/darwin/src/LookupTable1D_test_simple.csv changes by running from the arty directory:
```
uv --project ~/arty/scripts run ~/arty/scripts/gen_lookup_table_1d.py test_lut_simple 2 -5.0 5.0 10.0 tests/darwin/src/LookupTable1D/LookupTable1D_test_simple.csv tests/darwin/src/LookupTable1D/LookupTable1D_test_lut_simple.h
```
*/

#include "LookupTable1D.h"
#include <array>

constexpr int TEST_LUT_SIMPLE_X_LEN = 2;
constexpr float TEST_LUT_SIMPLE_X_MIN = -5.0000000000f;
constexpr float TEST_LUT_SIMPLE_X_MAX = 5.0000000000f;
constexpr float TEST_LUT_SIMPLE_X_GAP = 10.0000000000f;

constexpr std::array<float, TEST_LUT_SIMPLE_X_LEN> TEST_LUT_SIMPLE_BPS{-50.0000000000, 50.0000000000};

typedef LookupTable1D<TEST_LUT_SIMPLE_X_LEN, TEST_LUT_SIMPLE_X_MIN, TEST_LUT_SIMPLE_X_MAX, TEST_LUT_SIMPLE_X_GAP, TEST_LUT_SIMPLE_BPS> TestLutSimple;
