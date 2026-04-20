#pragma once
#include <algorithm>
#include <limits>
#include <cmath>
#include "math_util.h"
#ifdef abs
#undef abs // allow std::abs despite Arduino macro
#endif

class PID
{
public:
    PID(float kp, float ki, float kd,
        float min_out = -1e6f, float max_out = 1e6f,
        float min_integral = -1e6f, float max_integral = 1e6f,
        float integral_zone = std::numeric_limits<float>::infinity(),
        float deriv_lowpass_hz = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd)
    {
        if (min_out != -1e6f || max_out != 1e6f) {
            setOutputLimits(min_out, max_out);
        }
        if (min_integral != -1e6f || max_integral != 1e6f) {
            setIntegralLimits(min_integral, max_integral);
        }
        if (integral_zone != std::numeric_limits<float>::infinity()) {
            setIntegralZone(integral_zone);
        }
        if (deriv_lowpass_hz > 0.0f) {
            setDerivativeLowPass(deriv_lowpass_hz);
        }
    }




    // Calculate control output given a setpoint and a measurement.
    // dt is computed internally from steady_clock;
    // on the first call, derivative and integral are not applied to avoid a large transient.

    float calculate(float setpoint, float measurement, float measurement_d, float dt)
    {


        if (std::isnan(prev_meas_))
        {

            prev_meas_ = measurement;
            // First call: just proportional action.
            const float error = setpoint - measurement;
            return clampOutput(kp_ * error);
        }

        const float error = setpoint - measurement;

        // Integral (respect integral zone if enabled)
        if (!use_integral_zone_ || std::abs(error) <= integral_zone_)
        {
            integral_ += error * dt;
        }

        // Derivative on measurement to reduce derivative kick
        float deriv_raw = 0.0f;
        if (!std::isnan(prev_meas_))
        {
            deriv_raw = -measurement_d / std::max(dt, 1e-9f); // negative sign: d(error)/dt = -d(meas)/dt
        }
        prev_meas_ = measurement;

        // Optional 1st-order low-pass on derivative: y += a*(x - y)
        float deriv_term = deriv_raw;
        if (use_deriv_lp_)
        {
            const float rc = 1.0f / (2.0f * pi * std::max(deriv_cutoff_hz_, 1e-6f));
            const float a = dt / (rc + dt);
            deriv_state_ += a * (deriv_raw - deriv_state_);
            deriv_term = deriv_state_;
        }

        float integral_output = integral_ * ki_;
        if (use_integral_limits_)
        {
            integral_output = std::clamp(integral_output, min_integral_, max_integral_);
        }
        // prevent division by 0
        if (ki_ > 0.000001f)
        {
            integral_ = integral_output / ki_; // store back clamped integral
        }
        float output = kp_ * error + integral_output + kd_ * deriv_term;

        return clampOutput(output);
    }

    // uses previous measurement to calculate derivative
    float calculate(float setpoint, float measurement, float dt)
    {
        // Derivative on measurement to reduce derivative kick
        float dmeas = 0.0f;
        if (!std::isnan(prev_meas_))
        {
            dmeas = measurement - prev_meas_;
        }

        return calculate(setpoint, measurement, dmeas, dt);

    }

    // Reset internal state (integral, derivative filter). Optionally set a new integral value.
    void reset(float integral = 0.0f)
    {
        integral_ = integral;
        prev_meas_ = std::numeric_limits<float>::quiet_NaN();
        deriv_state_ = 0.0f;
    }

    // --- Configuration helpers ---
    void setGains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        reset();
    }

    void setP(float kp) { kp_ = kp; }
    void setI(float ki) { ki_ = ki; }
    void setD(float kd) { kd_ = kd; }

    float getP() const { return kp_; }
    float getI() const { return ki_; }
    float getD() const { return kd_; }
    float getIntegralOutput() const { return integral_ * ki_; }

    void setOutputLimits(float min_out, float max_out)
    {
        min_out_ = min_out;
        max_out_ = max_out;
        use_output_limits_ = true;
        if (min_out_ > max_out_)
            std::swap(min_out_, max_out_);
    }
    void clearOutputLimits() { use_output_limits_ = false; }

    void setIntegralLimits(float min_i, float max_i)
    {
        min_integral_ = min_i;
        max_integral_ = max_i;
        use_integral_limits_ = true;
        if (min_integral_ > max_integral_)
            std::swap(min_integral_, max_integral_);
    }
    void clearIntegralLimits() { use_integral_limits_ = false; }

    void setIntegralZone(float zone_abs_error)
    {
        integral_zone_ = std::max(0.0f, zone_abs_error);
        use_integral_zone_ = true;
    }
    void clearIntegralZone() { use_integral_zone_ = false; }

    // Enable a lowâ€‘pass filter on the derivative term. Example: cutoff_hz = 30.0
    void setDerivativeLowPass(float cutoff_hz)
    {
        deriv_cutoff_hz_ = cutoff_hz;
        use_deriv_lp_ = true;
    }
    void clearDerivativeLowPass()
    {
        use_deriv_lp_ = false;
        deriv_state_ = 0.0f;
    }

private:
    float clampOutput(float u) const
    {
        if (!use_output_limits_)
            return u;
        if (u > max_out_)
            return max_out_;
        if (u < min_out_)
            return min_out_;
        return u;
    }

    // Gains
    float kp_{};
    float ki_{};
    float kd_{};

    // State
    const float pi = std::acos(-1.0);

    float integral_ = 0.0f;
    float prev_meas_ = std::numeric_limits<float>::quiet_NaN();

    // Derivative lowâ€‘pass state
    bool use_deriv_lp_ = false;
    float deriv_cutoff_hz_ = 0.0f;
    float deriv_state_ = 0.0f;


    // Limits
    bool use_output_limits_ = false;
    float min_out_ = -1.0f, max_out_ = 1.0f;
    bool use_integral_limits_ = false;
    float min_integral_ = -1e6f, max_integral_ = 1e6f;
    bool use_integral_zone_ = false;
    float integral_zone_ = std::numeric_limits<float>::infinity();
};
