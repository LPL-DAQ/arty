// imu.cpp - BNO08x IMU wrapper using Arduino-Core-Zephyr

#include "imu.h"

// Zephyr logging
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#include <math.h>     // atan2f, asinf, acosf, cosf, sinf, sqrtf
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>      // sh2_SensorValue_t, sh2_saveDcdNow, SH2_ROTATION_VECTOR, SH2_OK


// ----- Euler ctor -----------------------------------------------------------

Euler::Euler(float yawDeg, float pitchDeg, float rollDeg)
    : yaw(yawDeg), pitch(pitchDeg), roll(rollDeg) {}

// ----- IMU public API -------------------------------------------------------
static constexpr uint8_t BNO_CS_PIN  = 10;  // Teensy CS
static constexpr uint8_t BNO_INT_PIN = 9;   // Your chosen INT pin
static constexpr uint8_t BNO_RST_PIN = 8;   // Your chosen RST pin

bool IMU::imu_init()
{
    LOG_INF("Initializing IMU");

    if (!_bno) {
        _bno = new Adafruit_BNO08x(BNO_RST_PIN);
        if (!_bno) {
            LOG_ERR("Failed to allocate Adafruit_BNO08x");
            return false;
        }
    }



    // Init SPI on the Arduino SPI bus (pins 10/11/12/13 on Teensy 4.1)
    SPI.begin();

    // Use the SPI begin instead of I2C
    if (!_bno->begin_SPI(BNO_CS_PIN, BNO_INT_PIN, &SPI)) {
        LOG_ERR("BNO08x begin_SPI failed");
        return false;
    }

    LOG_INF("BNO08x SPI init ok, enabling reports");

    // Enable the reports this application consumes
    _bno->enableReport(SH2_ROTATION_VECTOR, 10000);
    LOG_INF("IMU report enabled");

    LOG_INF("Finished IMU init");

    return true;
}

void IMU::imu_update()
{
    if (!_bno) {
        return;
    }

    if (_bno->wasReset())
    {
      Serial.println(F("[fly] IMU was reset."));
      _calibrationSaved        = false;
      _calibrationHighSinceMs  = 0;
      _lastCalSaveAttemptMs    = 0;
      _rvAccuracy              = 0;
      _rvAccuracyRad           = 0.0f;
    }

    sh2_SensorValue_t val;
    while (_bno->getSensorEvent(&val))
    {
      switch (val.sensorId)
      {
      case SH2_ROTATION_VECTOR:
        _quat.w = val.un.rotationVector.real;
        _quat.x = val.un.rotationVector.i;
        _quat.y = val.un.rotationVector.j;
        _quat.z = val.un.rotationVector.k;

        _quat       = normalize(_quat);
        _bodyQuat   = applyMountingCorrection(_quat);
        _bodyAxes   = quatToBodyAxes(_bodyQuat);
        _projectedAngles = computeProjectedAngles(_bodyAxes);
        _euler      = quatToEulerDeg(_bodyQuat);

        _rvAccuracyRad = val.un.rotationVector.accuracy;
        _rvAccuracy    = val.status & 0x03;
        _hasData       = true;
        break;

      default:
        break;
      }
    }

    if (_rvAccuracy >= 3)
    {
      if (_calibrationHighSinceMs == 0) {
        _calibrationHighSinceMs = millis();
      }
    }
    else
    {
      _calibrationHighSinceMs = 0;
      if (_rvAccuracy <= 1) {
        _calibrationSaved = false;
      }
    }

    const std::uint32_t now = millis();
    if (!_calibrationSaved && _calibrationHighSinceMs != 0)
    {
      constexpr std::uint32_t kMinStableMs = 3000;
      constexpr std::uint32_t kRetryMs     = 1000;
      bool stableLongEnough = (now - _calibrationHighSinceMs) >= kMinStableMs;
      bool retryElapsed     = (now - _lastCalSaveAttemptMs) >= kRetryMs;

      if (stableLongEnough && retryElapsed)
      {
        if (saveCalibration())
        {
          Serial.println(F("[fly] IMU calibration saved."));
          _calibrationHighSinceMs = 0;
        }
        else
        {
          Serial.println(F("[fly] IMU calibration save failed, will retry."));
          _lastCalSaveAttemptMs = now;
        }
      }
    }
}

bool IMU::hasData() const { return _hasData; }
QuaternionF IMU::quat() const { return _quat; }
QuaternionF IMU::bodyQuat() const { return _bodyQuat; }
Euler IMU::euler() const { return _euler; }

LinearAccel IMU::accelerometer() const { return _accelWorld; }
LinearAccel IMU::accelerometerWorld() const { return _accelWorld; }
LinearAccel IMU::accelerometerBody() const { return _accelBody; }
LinearAccel IMU::accelerometerSensor() const { return _rawAccel; }

std::uint8_t IMU::rotationAccuracy() const { return _rvAccuracy; }
float IMU::rotationAccuracyRad() const { return _rvAccuracyRad; }
bool IMU::isFullyCalibrated() const { return _rvAccuracy >= 3; }

BodyAxesF IMU::bodyAxes() const { return _bodyAxes; }
ProjectedAngles IMU::projectedAngles() const { return _projectedAngles; }

void IMU::setSensorToBodyEuler(float yawDeg, float pitchDeg, float rollDeg)
{
    _sensorToBody = normalize(quatFromEulerDeg(yawDeg, pitchDeg, rollDeg));
}

void IMU::setSensorToBodyQuat(const QuaternionF &q)
{
    _sensorToBody = normalize(q);
}

bool IMU::saveCalibration()
{
    const int rc = sh2_saveDcdNow();
    _lastCalSaveAttemptMs = millis();
    if (rc == SH2_OK)
    {
      _calibrationSaved = true;
      return true;
    }
    return false;
}

float IMU::yawDeg() const { return _euler.yaw; }
float IMU::pitchDeg() const { return _euler.pitch; }
float IMU::rollDeg() const { return _euler.roll; }

// ----- Static helpers -------------------------------------------------------

Euler IMU::quatToEulerDeg(const QuaternionF &q)
{
    const float ysqr = q.y * q.y;

    float t0 = +2.0f * (q.w * q.x + q.y * q.z);
    float t1 = +1.0f - 2.0f * (q.x * q.x + ysqr);
    float roll = atan2f(t0, t1);

    float t2 = +2.0f * (q.w * q.y - q.z * q.x);
    t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
    float pitch = asinf(t2);

    float t3 = +2.0f * (q.w * q.z + q.x * q.y);
    float t4 = +1.0f - 2.0f * (ysqr + q.z * q.z);
    float yaw = atan2f(t3, t4);

    return Euler{yaw * RAD2DEG, pitch * RAD2DEG, roll * RAD2DEG};
}

QuaternionF IMU::normalize(const QuaternionF &q)
{
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0f)
      return QuaternionF{};
    const float inv = 1.0f / norm;
    return QuaternionF{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
}

QuaternionF IMU::conjugate(const QuaternionF &q)
{
    return QuaternionF{q.w, -q.x, -q.y, -q.z};
}

QuaternionF IMU::multiply(const QuaternionF &a, const QuaternionF &b)
{
    return QuaternionF{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

QuaternionF IMU::quatFromEulerDeg(float yawDeg, float pitchDeg, float rollDeg)
{
    const float cy = cosf(0.5f * yawDeg * DEG2RAD);
    const float sy = sinf(0.5f * yawDeg * DEG2RAD);
    const float cp = cosf(0.5f * pitchDeg * DEG2RAD);
    const float sp = sinf(0.5f * pitchDeg * DEG2RAD);
    const float cr = cosf(0.5f * rollDeg * DEG2RAD);
    const float sr = sinf(0.5f * rollDeg * DEG2RAD);

    QuaternionF q{
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr};
    return normalize(q);
}

void IMU::rotateVector(const QuaternionF &q,
                       float x, float y, float z,
                       float &ox, float &oy, float &oz)
{
    const QuaternionF vec{0.0f, x, y, z};
    const QuaternionF qc  = conjugate(q);
    const QuaternionF tmp = multiply(q, vec);
    const QuaternionF res = multiply(tmp, qc);
    ox = res.x;
    oy = res.y;
    oz = res.z;
}

float IMU::clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

BodyAxesF IMU::quatToBodyAxes(const QuaternionF &q)
{
    BodyAxesF axes{};
    rotateVector(q, 1.0f, 0.0f, 0.0f, axes.forwardX, axes.forwardY, axes.forwardZ);
    rotateVector(q, 0.0f, 1.0f, 0.0f, axes.rightX,  axes.rightY,  axes.rightZ);
    rotateVector(q, 0.0f, 0.0f, 1.0f, axes.upX,     axes.upY,     axes.upZ);
    return axes;
}

ProjectedAngles IMU::computeProjectedAngles(const BodyAxesF &axes)
{
    ProjectedAngles proj{};
    proj.tiltAboutX = atan2f(-axes.upY, axes.upZ) * RAD2DEG;
    proj.tiltAboutY = atan2f(axes.upX,  axes.upZ) * RAD2DEG;
    proj.totalTilt  = acosf(clampf(axes.upZ, -1.0f, 1.0f)) * RAD2DEG;

    const float heading = atan2f(axes.forwardY, axes.forwardX);
    proj.heading = heading * RAD2DEG;
    return proj;
}

QuaternionF IMU::applyMountingCorrection(const QuaternionF &sensorQuat) const
{
    return normalize(multiply(sensorQuat, conjugate(_sensorToBody)));
}
