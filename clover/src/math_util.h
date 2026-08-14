#ifndef APP_MATH_H
#define APP_MATH_H
#include <cstdint>
#include <cmath>
#include <algorithm>


#include "clover.pb.h"

namespace math_util
{

  inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  // degrees ↔ microseconds (generic 1000–2000 µs)
  inline uint16_t angleDegToUs(float deg, float minDeg, float maxDeg, uint16_t minUs, uint16_t maxUs)
  {
    deg = std::clamp(deg, minDeg, maxDeg);
    return static_cast<uint16_t>(mapFloat(deg, minDeg, maxDeg, minUs, maxUs));
  }

  inline Quaternion createQuaternion(float qw, float qx, float qy, float qz)
  {
      Quaternion out;
      out.qw = qw;
      out.qx = qx;
      out.qy = qy;
      out.qz = qz;
      return out;
  }

  inline Vector3D createVector3D(float x, float y, float z)
  {
      Vector3D out;
      out.x = x;
      out.y = y;
      out.z = z;
      return out;
  }

  inline Vector3D unitZ()
  {
      return createVector3D(0.0f, 0.0f, 1.0f);
  }

  inline Vector3D quaternionToEulerAngles(const Quaternion& q)
  {
      // Convert quaternion to euler angles (yaw, pitch, roll)
      // Using the same convention as Eigen: eulerAngles(2, 1, 0) which is yaw, pitch, roll
      const float qw = q.qw;
      const float qx = q.qx;
      const float qy = q.qy;
      const float qz = q.qz;

      // Roll (x-axis rotation)
      const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
      const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
      const float roll = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch (y-axis rotation)
      const float sinp = 2.0f * (qw * qy - qz * qx);
      const float pi = 3.14159265358979323846f;
      const float pitch = std::abs(sinp) >= 1.0f ? std::copysignf(pi / 2.0f, sinp) : std::asin(sinp);

      // Yaw (z-axis rotation)
      const float siny_cosp = 2.0f * (qw * qz + qx * qy);
      const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
      const float yaw = std::atan2(siny_cosp, cosy_cosp);

      return createVector3D(yaw, pitch, roll);
  }

      inline Quaternion normalizeQuaternion(const Quaternion& q)
    {
        const float norm =
            std::sqrt(q.qw * q.qw +
                      q.qx * q.qx +
                      q.qy * q.qy +
                      q.qz * q.qz);

        Quaternion out;

        // Avoid divide-by-zero / NaN
        if (norm <= 1e-8f)
        {
            out.qw = 1.0f;
            out.qx = 0.0f;
            out.qy = 0.0f;
            out.qz = 0.0f;
            return out;
        }

        out.qw = q.qw / norm;
        out.qx = q.qx / norm;
        out.qy = q.qy / norm;
        out.qz = q.qz / norm;
        return out;
    }

    inline Quaternion conjugateQuaternion(const Quaternion& q)
    {
        Quaternion out;
        out.qw = q.qw;
        out.qx = -q.qx;
        out.qy = -q.qy;
        out.qz = -q.qz;
        return out;
    }

    inline Vector3D normalizeVector3D(const Vector3D& v)
    {
        const float norm = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

        Vector3D out;

        // Avoid divide-by-zero / NaN
        if (norm <= 1e-8f)
        {
            out.x = 0.0f;
            out.y = 0.0f;
            out.z = 0.0f;  // Default to unit Z if zero vector
            return out;
        }

        out.x = v.x / norm;
        out.y = v.y / norm;
        out.z = v.z / norm;
        return out;
    }
    // Internal helper: cross product
    inline Vector3D crossProduct(const Vector3D& a, const Vector3D& b)
    {
        Vector3D out;
        out.x = a.y * b.z - a.z * b.y;
        out.y = a.z * b.x - a.x * b.z;
        out.z = a.x * b.y - a.y * b.x;
        return out;
    }

    // Rotate a 3D vector by a quaternion:
    // v_rot = q * v * q_conj
    //
    // This version uses the efficient vector form:
    // t = 2 * cross(q_vec, v)
    // v' = v + qw * t + cross(q_vec, t)
    //
    // Assumes q represents the rotation you want to apply.
    inline Vector3D multiplyQuaternionVector(const Quaternion& q_in, const Vector3D& v)
    {
        const Quaternion q = normalizeQuaternion(q_in);

        Vector3D q_vec;
        q_vec.x = q.qx;
        q_vec.y = q.qy;
        q_vec.z = q.qz;

        Vector3D t = crossProduct(q_vec, v);
        t.x = 2.0f * t.x;
        t.y = 2.0f * t.y;
        t.z = 2.0f * t.z;

        Vector3D cross_q_t = crossProduct(q_vec, t);

        Vector3D out;
        out.x = v.x + q.qw * t.x + cross_q_t.x;
        out.y = v.y + q.qw * t.y + cross_q_t.y;
        out.z = v.z + q.qw * t.z + cross_q_t.z;

        return out;
    }

    // Do two quaternions agree to within threshold, compared per component?
    inline bool quaternionsAgree(
        float aw, float ax, float ay, float az, float bw, float bx, float by, float bz, float threshold
    )
    {
        return std::fabs(aw - bw) < threshold && std::fabs(ax - bx) < threshold && std::fabs(ay - by) < threshold &&
               std::fabs(az - bz) < threshold;
    }

    // Do two 3-vectors agree to within threshold, compared per axis?
    inline bool vectorsClose(float ax, float ay, float az, float bx, float by, float bz, float threshold)
    {
        return std::fabs(ax - bx) < threshold && std::fabs(ay - by) < threshold && std::fabs(az - bz) < threshold;
    }

    // ── 3x3 matrix / 3-vector helpers ───────────────────────────────────────────────────────
    // Out-parameter style since raw C arrays can't be returned by value. These operate on plain
    // float arrays rather than Quaternion/Vector3D, so they don't overlap with the vector helpers
    // above.

    inline void multiplyMatrix3(const float a[3][3], const float b[3][3], float out[3][3])
    {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++) {
                    sum += a[i][k] * b[k][j];
                }
                out[i][j] = sum;
            }
        }
    }

    inline void transposeMatrix3(const float a[3][3], float out[3][3])
    {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out[j][i] = a[i][j];
            }
        }
    }

    inline void addMatrix3(const float a[3][3], const float b[3][3], float out[3][3])
    {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out[i][j] = a[i][j] + b[i][j];
            }
        }
    }

    inline void multiplyMatrix3Vector(const float a[3][3], const float v[3], float out[3])
    {
        for (int i = 0; i < 3; i++) {
            float sum = 0.0f;
            for (int j = 0; j < 3; j++) {
                sum += a[i][j] * v[j];
            }
            out[i] = sum;
        }
    }

    inline float dotProduct(const float a[3], const float b[3])
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    // Outer product out = a * b^T.
    inline void outerProduct(const float a[3], const float b[3], float out[3][3])
    {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out[i][j] = a[i] * b[j];
            }
        }
    }

    // In-place a = 0.5*(a + a^T). Guards against a covariance matrix drifting asymmetric from
    // float rounding across repeated updates.
    inline void symmetrizeMatrix3(float a[3][3])
    {
        float transposed[3][3];
        transposeMatrix3(a, transposed);

        float summed[3][3];
        addMatrix3(a, transposed, summed);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                a[i][j] = 0.5f * summed[i][j];
            }
        }
    }
}
#endif
