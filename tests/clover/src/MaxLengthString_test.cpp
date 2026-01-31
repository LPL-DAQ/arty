#include "MaxLengthString.h"
#include <zephyr/ztest.h>
#include <cstring>

ZTEST(MaxLengthString_test, test_default_constructor)
{
    MaxLengthString<5> str;

    zassert_equal(std::strcmp(str.c_str(), ""), 0);
    zassert_equal(str.string_view(), "");
    zassert_equal(str.size(), 0);
}

ZTEST(MaxLengthString_test, test_formatted_constructor)
{
    // Basic integer formatting
    {
        MaxLengthString<5> str("%d", -2);

        zassert_equal(std::strcmp(str.c_str(), "-2"), 0);
        zassert_equal(str.string_view(), "-2");
        zassert_equal(str.size(), 2);
    }

    {
        MaxLengthString<8> str("%d", 42112321);

        zassert_equal(std::strcmp(str.c_str(), "42112321"), 0);
        zassert_equal(str.string_view(), "42112321");
        zassert_equal(str.size(), 8);
    }

    {
        MaxLengthString<8> str("%d", 0);

        zassert_equal(std::strcmp(str.c_str(), "0"), 0);
        zassert_equal(str.string_view(), "0");
        zassert_equal(str.size(), 1);
    }

    // Basic string formatting
    {
        MaxLengthString<5> str("%s", "-2");

        zassert_equal(std::strcmp(str.c_str(), "-2"), 0);
        zassert_equal(str.string_view(), "-2");
        zassert_equal(str.size(), 2);
    }

    {
        MaxLengthString<25> str("%s", "fnjcxvklnm,fkjdashjkfdhas");

        zassert_equal(std::strcmp(str.c_str(), "fnjcxvklnm,fkjdashjkfdhas"), 0);
        zassert_equal(str.string_view(), "fnjcxvklnm,fkjdashjkfdhas");
        zassert_equal(str.size(), 25);
    }

    // Basic float formatting
    {
        MaxLengthString<8> str("%f.2f", 32.5);

        zassert_equal(str.string_view(), "32.50");
        zassert_equal(std::strcmp(str.c_str(), "32.50"), 0);
        zassert_equal(str.size(), 5);
    }

    // Multiple formatters
    {
        MaxLengthString<60> str("%d%d%s%f.1f%svncxmz,vnmz,cxnvm,czn,mvzx%d%f.3f%s", 2, 3, "hey", 4.5, "what", -2323, 9.999, "bruhhhh");

        zassert_equal(std::strcmp(str.c_str(), "23hey4.5whatvncxmz,vnmz,cxnvm,czn,mvzx-23239.999bruhhh"), 0);
        zassert_equal(str.string_view(), "23hey4.5whatvncxmz,vnmz,cxnvm,czn,mvzx-23239.999bruhhh");
        zassert_equal(str.size(), 54);
    }
}

ZTEST_SUITE(MaxLengthString_test, NULL, NULL, NULL, NULL, NULL);
