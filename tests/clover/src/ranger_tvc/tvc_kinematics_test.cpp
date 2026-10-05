#include "ranger/tvc/tvc_kinematics.h"
#include "tvc_ik_vectors.h"

#include <cmath>
#include <initializer_list>
#include <zephyr/ztest.h>

using namespace tvc;

namespace {
constexpr float IK_TOLERANCE_IN = 1e-4f;
}

ZTEST(TvcKinematics_tests, test_pitch_matches_matlab_csv)
{
    for (const auto& v : TVC_PITCH_IK_VECTORS) {
        const float l = length_in(TVC_PITCH, v.angle_deg);
        zassert_within(l, v.length_in, IK_TOLERANCE_IN, "pitch %.2f deg: got %.6f in, expected %.6f in", (double)v.angle_deg, (double)l, (double)v.length_in);
    }
}

ZTEST(TvcKinematics_tests, test_yaw_matches_matlab_formula)
{
    for (const auto& v : TVC_YAW_IK_VECTORS) {
        const float l = length_in(TVC_YAW, v.angle_deg);
        zassert_within(l, v.length_in, IK_TOLERANCE_IN, "yaw %.2f deg: got %.6f in, expected %.6f in", (double)v.angle_deg, (double)l, (double)v.length_in);
    }
}

ZTEST(TvcKinematics_tests, test_reference_values_from_prompt)
{
    zassert_within(length_in(TVC_PITCH, 0.0f), 18.3582f, IK_TOLERANCE_IN);
    zassert_within(length_in(TVC_PITCH, -12.0f), 17.0789f, IK_TOLERANCE_IN);
    zassert_within(length_in(TVC_PITCH, 12.0f), 19.4677f, IK_TOLERANCE_IN);
}

ZTEST(TvcKinematics_tests, test_l_center_constants_match_ik)
{
    for (const AxisConfig* axis : TVC_AXES) {
        zassert_within(axis->l_center_in, length_in(*axis, 0.0f), IK_TOLERANCE_IN, "%s l_center_in disagrees with IK at 0 deg", axis->name);
    }
}

ZTEST(TvcKinematics_tests, test_angle_is_inverse_of_length)
{
    for (const AxisConfig* axis : TVC_AXES) {
        for (float a = -12.0f; a <= 12.0f; a += 0.25f) {
            const float back = angle_deg(*axis, length_in(*axis, a));
            zassert_within(back, a, 2e-3f, "%s: %.3f deg round-tripped to %.5f deg", axis->name, (double)a, (double)back);
        }
    }
}

ZTEST(TvcKinematics_tests, test_length_is_monotonic_increasing)
{
    for (const AxisConfig* axis : TVC_AXES) {
        float prev = length_in(*axis, -12.0f);
        for (float a = -11.9f; a <= 12.0f; a += 0.1f) {
            const float l = length_in(*axis, a);
            zassert_true(l > prev, "%s length not increasing at %.2f deg", axis->name, (double)a);
            prev = l;
        }
    }
}

ZTEST(TvcKinematics_tests, test_max_dl_dangle_bound_holds)
{
    for (const AxisConfig* axis : TVC_AXES) {
        for (float a = -12.0f; a < 12.0f; a += 0.1f) {
            const float slope = (length_in(*axis, a + 0.1f) - length_in(*axis, a)) / 0.1f;
            zassert_true(slope <= TVC_MAX_DL_DANGLE_IN_PER_DEG, "%s slope %.5f in/deg exceeds bound", axis->name, (double)slope);
        }
    }
}

ZTEST(TvcKinematics_tests, test_length_to_rev)
{
    // 0.2 in/rev lead screw, direct drive -> 5 turns per inch.
    zassert_within(length_to_rev(18.3582f, 18.3582f, 5.0f, 1.0f), 0.0f, 1e-6f);
    zassert_within(length_to_rev(18.5582f, 18.3582f, 5.0f, 1.0f), 1.0f, 1e-4f);
    zassert_within(length_to_rev(18.5582f, 18.3582f, 5.0f, -1.0f), -1.0f, 1e-4f);
    zassert_within(length_to_rev(17.0789f, 18.3582f, 5.0f, 1.0f), -6.3965f, 1e-3f);
}

ZTEST(TvcKinematics_tests, test_rev_to_length_is_inverse)
{
    for (float sign : {1.0f, -1.0f}) {
        for (float l = 17.0f; l <= 19.5f; l += 0.05f) {
            const float rev = length_to_rev(l, 18.3582f, 7.5f, sign);
            zassert_within(rev_to_length(rev, 18.3582f, 7.5f, sign), l, 1e-5f);
        }
    }
}

ZTEST(TvcKinematics_tests, test_axis_config_completeness)
{
    AxisConfig axis = TVC_PITCH;
    axis.turns_per_inch = 5.0f;
    axis.direction_sign = -1.0f;
    zassert_true(axis_config_is_complete(axis));

    axis.turns_per_inch = 0.0f;  // Placeholder
    zassert_false(axis_config_is_complete(axis));

    axis.turns_per_inch = 5.0f;
    axis.direction_sign = 0.0f;  // Placeholder
    zassert_false(axis_config_is_complete(axis));

    axis.direction_sign = 1.0f;
    axis.moteus_id = 0x80;  // Not a valid 7-bit ID
    zassert_false(axis_config_is_complete(axis));
}

ZTEST_SUITE(TvcKinematics_tests, NULL, NULL, NULL, NULL, NULL);
