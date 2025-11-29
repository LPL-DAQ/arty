// pid.h
#pragma once

#include <limits>
#include <cmath>
#include "math.h"

#ifdef abs
#undef abs // allow std::abs despite Arduino macro
#endif

class PID
{
public:
    PID(double kp, double ki, double kd);

    // Calculate control output given a setpoint and a measurement.
    // dt is the timestep (seconds).
    double calculate(double setpoint, double measurement, double dt);

    // Reset internal state (integral, derivative filter). Optionally set a new integral value.
    void reset(double integral = 0.0);

    // --- Configuration helpers ---
    void setGains(double kp, double ki, double kd);
    void setP(double kp);
    void setI(double ki);
    void setD(double kd);

    double getP() const;
    double getI() const;
    double getD() const;

    void setOutputLimits(double min_out, double max_out);
    void clearOutputLimits();

    void setIntegralLimits(double min_i, double max_i);
    void clearIntegralLimits();

    void setIntegralZone(double zone_abs_error);
    void clearIntegralZone();

    // Enable a low-pass filter on the derivative term. Example: cutoff_hz = 30.0
    void setDerivativeLowPass(double cutoff_hz);
    void clearDerivativeLowPass();

private:
    double clampOutput(double u) const;

    // Gains
    double kp_{};
    double ki_{};
    double kd_{};

    // State
    double integral_ = 0.0;
    double prev_meas_ = std::numeric_limits<double>::quiet_NaN();

    // Derivative low-pass state
    bool use_deriv_lp_ = false;
    double deriv_cutoff_hz_ = 0.0;
    double deriv_state_ = 0.0;

    // Limits
    bool use_output_limits_ = false;
    double min_out_ = -1.0, max_out_ = 1.0;
    bool use_integral_limits_ = false;
    double min_integral_ = -1e6, max_integral_ = 1e6;
    bool use_integral_zone_ = false;
    double integral_zone_ = std::numeric_limits<double>::infinity();
};
