#include "../../../../clover/src/math_util.h"
#include <zephyr/ztest.h>
#include <cmath>
#include <algorithm>


constexpr float EPSILON = 0.001f;

// ── normalizeQuaternion ──────────────────────────────────────────────────────

ZTEST(MathUtil_tests, test_normalize_unit_quaternion_unchanged)
{
    // A unit quaternion should come back essentially unchanged
    Quaternion q = math_util::createQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion n = math_util::normalizeQuaternion(q);
    zassert_within(n.qw, 1.0f, EPSILON, "qw should remain 1");
    zassert_within(n.qx, 0.0f, EPSILON, "qx should remain 0");
    zassert_within(n.qy, 0.0f, EPSILON, "qy should remain 0");
    zassert_within(n.qz, 0.0f, EPSILON, "qz should remain 0");
}

ZTEST(MathUtil_tests, test_normalize_scaled_quaternion)
{
    // Scale an identity quaternion by 5; should normalise back to identity
    Quaternion q = math_util::createQuaternion(5.0f, 0.0f, 0.0f, 0.0f);
    Quaternion n = math_util::normalizeQuaternion(q);
    float mag = std::sqrt(n.qw*n.qw + n.qx*n.qx + n.qy*n.qy + n.qz*n.qz);
    zassert_within(mag, 1.0f, EPSILON, "Normalised quaternion should have unit magnitude");
    zassert_within(n.qw, 1.0f, EPSILON, "qw should be 1 after normalisation");
}

ZTEST(MathUtil_tests, test_normalize_arbitrary_quaternion_has_unit_magnitude)
{
    Quaternion q = math_util::createQuaternion(2.0f, 1.0f, 0.5f, -1.5f);
    Quaternion n = math_util::normalizeQuaternion(q);
    float mag = std::sqrt(n.qw*n.qw + n.qx*n.qx + n.qy*n.qy + n.qz*n.qz);
    zassert_within(mag, 1.0f, EPSILON, "Arbitrary normalised quaternion should have unit magnitude");
}

ZTEST(MathUtil_tests, test_normalize_zero_quaternion_returns_identity)
{
    // Zero quaternion should not produce NaN; implementation falls back to identity
    Quaternion q = math_util::createQuaternion(0.0f, 0.0f, 0.0f, 0.0f);
    Quaternion n = math_util::normalizeQuaternion(q);
    zassert_false(std::isnan(n.qw), "qw should not be NaN for zero quaternion");
    zassert_false(std::isnan(n.qx), "qx should not be NaN for zero quaternion");
    zassert_false(std::isnan(n.qy), "qy should not be NaN for zero quaternion");
    zassert_false(std::isnan(n.qz), "qz should not be NaN for zero quaternion");
    zassert_within(n.qw, 1.0f, EPSILON, "Zero quaternion should fall back to identity qw=1");
}

// ── conjugateQuaternion ──────────────────────────────────────────────────────

ZTEST(MathUtil_tests, test_conjugate_negates_vector_part)
{
    Quaternion q = math_util::createQuaternion(0.5f, 1.0f, -2.0f, 3.0f);
    Quaternion c = math_util::conjugateQuaternion(q);
    zassert_within(c.qw,  0.5f, EPSILON, "Conjugate should preserve qw");
    zassert_within(c.qx, -1.0f, EPSILON, "Conjugate should negate qx");
    zassert_within(c.qy,  2.0f, EPSILON, "Conjugate should negate qy");
    zassert_within(c.qz, -3.0f, EPSILON, "Conjugate should negate qz");
}

ZTEST(MathUtil_tests, test_conjugate_of_conjugate_is_original)
{
    Quaternion q = math_util::createQuaternion(0.7f, 0.3f, -0.5f, 0.2f);
    Quaternion cc = math_util::conjugateQuaternion(math_util::conjugateQuaternion(q));
    zassert_within(cc.qw, q.qw, EPSILON, "Double conjugate qw should equal original");
    zassert_within(cc.qx, q.qx, EPSILON, "Double conjugate qx should equal original");
    zassert_within(cc.qy, q.qy, EPSILON, "Double conjugate qy should equal original");
    zassert_within(cc.qz, q.qz, EPSILON, "Double conjugate qz should equal original");
}

// ── multiplyQuaternionVector ─────────────────────────────────────────────────

ZTEST(MathUtil_tests, test_rotate_vector_by_identity_quaternion_unchanged)
{
    // Identity rotation should leave any vector unchanged
    Quaternion q = math_util::createQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3D v = math_util::createVector3D(1.0f, 2.0f, 3.0f);
    Vector3D r = math_util::multiplyQuaternionVector(q, v);
    zassert_within(r.x, 1.0f, EPSILON, "Identity rotation should not change x");
    zassert_within(r.y, 2.0f, EPSILON, "Identity rotation should not change y");
    zassert_within(r.z, 3.0f, EPSILON, "Identity rotation should not change z");
}

ZTEST(MathUtil_tests, test_rotate_unit_z_by_90deg_pitch_gives_unit_x)
{
    // 90-degree rotation about Y-axis: Z -> -X  (or X depending on convention).
    // axis=(0,1,0), angle=pi/2 → qw=cos(pi/4), qy=sin(pi/4)
    const float half = 3.14159265f / 4.0f;
    Quaternion q = math_util::createQuaternion(std::cos(half), 0.0f, std::sin(half), 0.0f);
    Vector3D z = math_util::createVector3D(0.0f, 0.0f, 1.0f);
    Vector3D r = math_util::multiplyQuaternionVector(q, z);
    // After 90° rotation about Y, Z becomes +X (right-hand rule)
    zassert_within(r.x,  1.0f, EPSILON, "90-deg Y rotation of Z should give +X component");
    zassert_within(r.y,  0.0f, EPSILON, "90-deg Y rotation of Z should give 0 Y component");
    zassert_within(r.z,  0.0f, EPSILON, "90-deg Y rotation of Z should give 0 Z component");
}

ZTEST(MathUtil_tests, test_rotate_vector_preserves_magnitude)
{
    // Rotation must not change vector length
    const float half = 3.14159265f / 6.0f; // 60 deg about X
    Quaternion q = math_util::createQuaternion(std::cos(half), std::sin(half), 0.0f, 0.0f);
    Vector3D v = math_util::createVector3D(1.0f, 2.0f, 3.0f);
    Vector3D r = math_util::multiplyQuaternionVector(q, v);
    float mag_v = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    float mag_r = std::sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
    zassert_within(mag_r, mag_v, EPSILON, "Rotation should preserve vector magnitude");
}

ZTEST(MathUtil_tests, test_rotate_vector_by_non_unit_quaternion_same_as_normalised)
{
    // Passing a scaled quaternion should give the same result as the normalised version
    Quaternion q_raw = math_util::createQuaternion(2.0f, 0.0f, 0.0f, 0.0f); // scaled identity
    Quaternion q_norm = math_util::normalizeQuaternion(q_raw);
    Vector3D v = math_util::createVector3D(3.0f, -1.0f, 2.0f);
    Vector3D r_raw  = math_util::multiplyQuaternionVector(q_raw,  v);
    Vector3D r_norm = math_util::multiplyQuaternionVector(q_norm, v);
    zassert_within(r_raw.x, r_norm.x, EPSILON, "Non-unit and unit quaternion rotation should agree in x");
    zassert_within(r_raw.y, r_norm.y, EPSILON, "Non-unit and unit quaternion rotation should agree in y");
    zassert_within(r_raw.z, r_norm.z, EPSILON, "Non-unit and unit quaternion rotation should agree in z");
}

// ── normalizeVector3D ────────────────────────────────────────────────────────

ZTEST(MathUtil_tests, test_normalize_vector3d_has_unit_length)
{
    Vector3D v = math_util::createVector3D(3.0f, 4.0f, 0.0f);
    Vector3D n = math_util::normalizeVector3D(v);
    float mag = std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
    zassert_within(mag, 1.0f, EPSILON, "normalizeVector3D should produce unit-length vector");
}

ZTEST(MathUtil_tests, test_normalize_zero_vector3d_no_nan)
{
    Vector3D v = math_util::createVector3D(0.0f, 0.0f, 0.0f);
    Vector3D n = math_util::normalizeVector3D(v);
    zassert_false(std::isnan(n.x), "Zero vector normalisation should not produce NaN x");
    zassert_false(std::isnan(n.y), "Zero vector normalisation should not produce NaN y");
    zassert_false(std::isnan(n.z), "Zero vector normalisation should not produce NaN z");
}

// ── crossProduct ─────────────────────────────────────────────────────────────

ZTEST(MathUtil_tests, test_cross_product_x_cross_y_is_z)
{
    Vector3D x = math_util::createVector3D(1.0f, 0.0f, 0.0f);
    Vector3D y = math_util::createVector3D(0.0f, 1.0f, 0.0f);
    Vector3D z = math_util::crossProduct(x, y);
    zassert_within(z.x, 0.0f, EPSILON, "X cross Y should have zero x component");
    zassert_within(z.y, 0.0f, EPSILON, "X cross Y should have zero y component");
    zassert_within(z.z, 1.0f, EPSILON, "X cross Y should equal Z");
}

ZTEST(MathUtil_tests, test_cross_product_anticommutative)
{
    Vector3D a = math_util::createVector3D(1.0f, 2.0f, 3.0f);
    Vector3D b = math_util::createVector3D(4.0f, -1.0f, 2.0f);
    Vector3D ab = math_util::crossProduct(a, b);
    Vector3D ba = math_util::crossProduct(b, a);
    zassert_within(ab.x, -ba.x, EPSILON, "Cross product should be anticommutative in x");
    zassert_within(ab.y, -ba.y, EPSILON, "Cross product should be anticommutative in y");
    zassert_within(ab.z, -ba.z, EPSILON, "Cross product should be anticommutative in z");
}

ZTEST(MathUtil_tests, test_cross_product_parallel_vectors_is_zero)
{
    Vector3D a = math_util::createVector3D(2.0f, 0.0f, 0.0f);
    Vector3D b = math_util::createVector3D(5.0f, 0.0f, 0.0f);
    Vector3D c = math_util::crossProduct(a, b);
    zassert_within(c.x, 0.0f, EPSILON, "Parallel cross product x should be 0");
    zassert_within(c.y, 0.0f, EPSILON, "Parallel cross product y should be 0");
    zassert_within(c.z, 0.0f, EPSILON, "Parallel cross product z should be 0");
}

// ── quaternion * conjugate = identity ────────────────────────────────────────

ZTEST(MathUtil_tests, test_quaternion_times_conjugate_is_identity)
{
    // For any unit quaternion q, Hamilton product q * conj(q) must equal (1,0,0,0).
    // Hamilton product: (a*w - b*x - c*y - d*z, a*x + b*w + c*z - d*y,
    //                    a*y - b*z + c*w + d*x,  a*z + b*y - c*x + d*w)
    // where q=(a,b,c,d) and r=(w,x,y,z).
    const float half = 3.14159265f / 6.0f; // 60 deg about Z
    Quaternion q = math_util::normalizeQuaternion(
        math_util::createQuaternion(std::cos(half), 0.0f, 0.0f, std::sin(half)));
    Quaternion c = math_util::conjugateQuaternion(q);

    // Hamilton product q * c
    float rw = q.qw*c.qw - q.qx*c.qx - q.qy*c.qy - q.qz*c.qz;
    float rx = q.qw*c.qx + q.qx*c.qw + q.qy*c.qz - q.qz*c.qy;
    float ry = q.qw*c.qy - q.qx*c.qz + q.qy*c.qw + q.qz*c.qx;
    float rz = q.qw*c.qz + q.qx*c.qy - q.qy*c.qx + q.qz*c.qw;

    zassert_within(rw, 1.0f, EPSILON, "q * conj(q) should give qw=1");
    zassert_within(rx, 0.0f, EPSILON, "q * conj(q) should give qx=0");
    zassert_within(ry, 0.0f, EPSILON, "q * conj(q) should give qy=0");
    zassert_within(rz, 0.0f, EPSILON, "q * conj(q) should give qz=0");
}

// ── rotate then unrotate recovers original vector ────────────────────────────

ZTEST(MathUtil_tests, test_rotate_then_conjugate_rotate_recovers_original_vector)
{
    // v' = q * v; v'' = conj(q) * v' should equal v
    const float half = 3.14159265f / 5.0f; // 72 deg about an arbitrary axis
    Quaternion q = math_util::normalizeQuaternion(
        math_util::createQuaternion(std::cos(half), 0.6f*std::sin(half), 0.8f*std::sin(half), 0.0f));
    Vector3D v = math_util::createVector3D(1.0f, -2.0f, 3.0f);

    Vector3D rotated   = math_util::multiplyQuaternionVector(q, v);
    Vector3D recovered = math_util::multiplyQuaternionVector(math_util::conjugateQuaternion(q), rotated);

    zassert_within(recovered.x, v.x, EPSILON, "Round-trip rotation should recover original x");
    zassert_within(recovered.y, v.y, EPSILON, "Round-trip rotation should recover original y");
    zassert_within(recovered.z, v.z, EPSILON, "Round-trip rotation should recover original z");
}

// ── quaternionToEulerAngles ──────────────────────────────────────────────────
// Convention: returns Vector3D(yaw, pitch, roll) — ZYX intrinsic (yaw about Z,
// pitch about Y, roll about X).

ZTEST(MathUtil_tests, test_euler_identity_quaternion_gives_zero_angles)
{
    // Identity quaternion must produce zero yaw, pitch and roll
    Quaternion q = math_util::createQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3D e = math_util::quaternionToEulerAngles(q);
    zassert_within(e.x, 0.0f, EPSILON, "Identity quat: yaw should be 0");
    zassert_within(e.y, 0.0f, EPSILON, "Identity quat: pitch should be 0");
    zassert_within(e.z, 0.0f, EPSILON, "Identity quat: roll should be 0");
}

ZTEST(MathUtil_tests, test_euler_pure_yaw_90deg)
{
    // 90-degree rotation about Z → yaw = π/2, pitch = 0, roll = 0
    const float half = 3.14159265f / 4.0f;
    Quaternion q = math_util::createQuaternion(std::cos(half), 0.0f, 0.0f, std::sin(half));
    Vector3D e = math_util::quaternionToEulerAngles(q);
    zassert_within(e.x, 3.14159265f / 2.0f, EPSILON, "Pure 90-deg yaw: yaw should be pi/2");
    zassert_within(e.y, 0.0f, EPSILON, "Pure 90-deg yaw: pitch should be 0");
    zassert_within(e.z, 0.0f, EPSILON, "Pure 90-deg yaw: roll should be 0");
}

ZTEST(MathUtil_tests, test_euler_pure_roll_90deg)
{
    // 90-degree rotation about X → yaw = 0, pitch = 0, roll = π/2
    const float half = 3.14159265f / 4.0f;
    Quaternion q = math_util::createQuaternion(std::cos(half), std::sin(half), 0.0f, 0.0f);
    Vector3D e = math_util::quaternionToEulerAngles(q);
    zassert_within(e.x, 0.0f, EPSILON, "Pure 90-deg roll: yaw should be 0");
    zassert_within(e.y, 0.0f, EPSILON, "Pure 90-deg roll: pitch should be 0");
    zassert_within(e.z, 3.14159265f / 2.0f, EPSILON, "Pure 90-deg roll: roll should be pi/2");
}

ZTEST(MathUtil_tests, test_euler_combined_rotation_round_trip)
{
    // Construct a quaternion from known ZYX euler angles (yaw=0.5, pitch=0.3, roll=0.2 rad),
    // convert back to euler and verify the original angles are recovered.
    // ZYX composition: q = q_yaw * q_pitch * q_roll
    const float yaw   = 0.5f;
    const float pitch = 0.3f;
    const float roll  = 0.2f;

    // Half-angles
    const float cy = std::cos(yaw   / 2.0f), sy = std::sin(yaw   / 2.0f);
    const float cp = std::cos(pitch / 2.0f), sp = std::sin(pitch / 2.0f);
    const float cr = std::cos(roll  / 2.0f), sr = std::sin(roll  / 2.0f);

    Quaternion q = math_util::createQuaternion(
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy);

    Vector3D e = math_util::quaternionToEulerAngles(q);
    zassert_within(e.x, yaw,   EPSILON, "Round-trip: yaw should be recovered");
    zassert_within(e.y, pitch, EPSILON, "Round-trip: pitch should be recovered");
    zassert_within(e.z, roll,  EPSILON, "Round-trip: roll should be recovered");
}

ZTEST(MathUtil_tests, test_euler_negative_yaw)
{
    // -90-degree rotation about Z → yaw = -π/2
    const float half = 3.14159265f / 4.0f;
    Quaternion q = math_util::createQuaternion(std::cos(half), 0.0f, 0.0f, -std::sin(half));
    Vector3D e = math_util::quaternionToEulerAngles(q);
    zassert_within(e.x, -3.14159265f / 2.0f, EPSILON, "Negative 90-deg yaw: yaw should be -pi/2");
    zassert_within(e.y, 0.0f, EPSILON, "Negative 90-deg yaw: pitch should be 0");
    zassert_within(e.z, 0.0f, EPSILON, "Negative 90-deg yaw: roll should be 0");
}

ZTEST_SUITE(MathUtil_tests, NULL, NULL, NULL, NULL, NULL);
