// pid.cpp
#include "pid.h"

double PID::calculate(double setpoint, double measurement, double dt)
{
    if (std::isnan(prev_meas_))
    {
        prev_meas_ = measurement;
        // First call: just proportional action.
        const double error = setpoint - measurement;
        return clampOutput(kp_ * error);
    }

    const double error = setpoint - measurement;

    // Integral (respect integral zone if enabled)
    if (!use_integral_zone_ || std::abs(error) <= integral_zone_)
    {
        integral_ += error * dt;
    }

    // Derivative on measurement to reduce derivative kick
    double deriv_raw = 0.0;
    if (!std::isnan(prev_meas_))
    {
        const double dmeas = measurement - prev_meas_;
        deriv_raw = -dmeas / std::max(dt, 1e-9); // d(error)/dt = -d(meas)/dt
    }
    prev_meas_ = measurement;

    // Optional 1st-order low-pass on derivative: y += a*(x - y)
    double deriv_term = deriv_raw;
    if (use_deriv_lp_)
    {
        const double rc = 1.0 / (2.0 * 6.283185 * std::max(deriv_cutoff_hz_, 1e-6));
        const double a = dt / (rc + dt);
        deriv_state_ += a * (deriv_raw - deriv_state_);
        deriv_term = deriv_state_;
    }

    double integral_output = integral_ * ki_;
    if (use_integral_limits_)
    {
        integral_output = util::clamp(integral_output, min_integral_, max_integral_);
    }
    // prevent division by 0
    if (ki_ > 0.000001)
    {
        integral_ = integral_output / ki_; // store back clamped integral
    }

    double output = kp_ * error + integral_output + kd_ * deriv_term;

    return clampOutput(output);
}

void PID::reset(double integral)
{
    integral_ = integral;
    prev_meas_ = std::numeric_limits<double>::quiet_NaN();
    deriv_state_ = 0.0;
}

PID::PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
}

void PID::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    reset();
}

void PID::setP(double kp) { kp_ = kp; }
void PID::setI(double ki) { ki_ = ki; }
void PID::setD(double kd) { kd_ = kd; }

double PID::getP() const { return kp_; }
double PID::getI() const { return ki_; }
double PID::getD() const { return kd_; }

void PID::setOutputLimits(double min_out, double max_out)
{
    min_out_ = min_out;
    max_out_ = max_out;
    use_output_limits_ = true;
    if (min_out_ > max_out_)
        std::swap(min_out_, max_out_);
}

void PID::clearOutputLimits()
{
    use_output_limits_ = false;
}

void PID::setIntegralLimits(double min_i, double max_i)
{
    min_integral_ = min_i;
    max_integral_ = max_i;
    use_integral_limits_ = true;
    if (min_integral_ > max_integral_)
        std::swap(min_integral_, max_integral_);
}

void PID::clearIntegralLimits()
{
    use_integral_limits_ = false;
}

void PID::setIntegralZone(double zone_abs_error)
{
    integral_zone_ = std::max(0.0, zone_abs_error);
    use_integral_zone_ = true;
}

void PID::clearIntegralZone()
{
    use_integral_zone_ = false;
}

void PID::setDerivativeLowPass(double cutoff_hz)
{
    deriv_cutoff_hz_ = cutoff_hz;
    use_deriv_lp_ = true;
}

void PID::clearDerivativeLowPass()
{
    use_deriv_lp_ = false;
    deriv_state_ = 0.0;
}

double PID::clampOutput(double u) const
{
    if (!use_output_limits_)
        return u;
    if (u > max_out_)
        return max_out_;
    if (u < min_out_)
        return min_out_;
    return u;
}
