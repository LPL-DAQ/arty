#ifndef CLOVER_IMU_H
#define CLOVER_IMU_H

#include <cstdint>

// Forward declaration to avoid pulling in heavy Adafruit headers here.
class Adafruit_BNO08x;

/**
 * Simple IMU interface around a BNO08x device.
 * Provides orientation (quaternion, Euler), body axes, and accel data.
 */

struct LinearAccel
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f}; // m/s^2
};

struct Euler
{
  float yaw{0.0f};   // degrees
  float pitch{0.0f}; // degrees
  float roll{0.0f};  // degrees

  Euler() = default;
  Euler(float yawDeg, float pitchDeg, float rollDeg);
};

struct QuaternionF
{
  float w{1.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f}; // unit quaternion
};

struct BodyAxesF
{
  float forwardX{1.0f}, forwardY{0.0f}, forwardZ{0.0f};
  float rightX{0.0f},  rightY{1.0f},  rightZ{0.0f};
  float upX{0.0f},     upY{0.0f},     upZ{1.0f};
};

struct ProjectedAngles
{
  float tiltAboutX{0.0f}; // deg, rotation about body X (roll)
  float tiltAboutY{0.0f}; // deg, rotation about body Y (pitch)
  float totalTilt{0.0f};  // deg from vertical
  float heading{0.0f};    // deg, yaw derived from forward vector projection
};

class IMU
{
public:
  // Initialize IMU hardware and enable the rotation-vector report.
  bool imu_init();

  // Polls the IMU and updates orientation/accel state.
  void imu_update();

  // Accessors for current state.
  bool hasData() const;
  QuaternionF quat() const;
  QuaternionF bodyQuat() const;
  Euler euler() const;

  LinearAccel accelerometer() const;
  LinearAccel accelerometerWorld() const;
  LinearAccel accelerometerBody() const;
  LinearAccel accelerometerSensor() const;

  uint8_t rotationAccuracy() const;
  float rotationAccuracyRad() const;
  bool isFullyCalibrated() const;

  BodyAxesF bodyAxes() const;
  ProjectedAngles projectedAngles() const;

  // Mounting correction configuration.
  void setSensorToBodyEuler(float yawDeg, float pitchDeg, float rollDeg);
  void setSensorToBodyQuat(const QuaternionF &q);

  // Persist calibration to NVM (if supported by the device).
  bool saveCalibration();

  // Convenience getters for Euler components.
  float yawDeg() const;
  float pitchDeg() const;
  float rollDeg() const;

private:
  static constexpr float RAD2DEG = 57.2957795f;
  static constexpr float DEG2RAD = 0.0174532925f;

  // Quaternion / math helpers (implemented in imu.cpp).
  static Euler        quatToEulerDeg(const QuaternionF &q);
  static QuaternionF  normalize(const QuaternionF &q);
  static QuaternionF  conjugate(const QuaternionF &q);
  static QuaternionF  multiply(const QuaternionF &a, const QuaternionF &b);
  static QuaternionF  quatFromEulerDeg(float yawDeg, float pitchDeg, float rollDeg);
  static void         rotateVector(const QuaternionF &q,
                                   float x, float y, float z,
                                   float &ox, float &oy, float &oz);
  static float        clampf(float v, float lo, float hi);
  static BodyAxesF    quatToBodyAxes(const QuaternionF &q);
  static ProjectedAngles computeProjectedAngles(const BodyAxesF &axes);

  QuaternionF applyMountingCorrection(const QuaternionF &sensorQuat) const;

  // State
  Adafruit_BNO08x *_bno{nullptr};   // or value-type if you prefer; match imu.cpp

  QuaternionF   _quat{};
  QuaternionF   _bodyQuat{};
  Euler         _euler{};
  LinearAccel   _rawAccel{};
  LinearAccel   _accelBody{};
  LinearAccel   _accelWorld{};
  bool          _hasData{false};
  uint8_t       _rvAccuracy{0};
  float         _rvAccuracyRad{0.0f};
  bool          _calibrationSaved{false};
  std::uint32_t _calibrationHighSinceMs{0};
  std::uint32_t _lastCalSaveAttemptMs{0};
  QuaternionF   _sensorToBody{};
  BodyAxesF     _bodyAxes{};
  ProjectedAngles _projectedAngles{};
};

#endif // CLOVER_IMU_H
