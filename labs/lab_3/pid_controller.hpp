#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>
#include <cmath>

namespace Automation {

class PIDController {
public:
    struct Params {
        double Kp{1.0};
        double Ki{0.0};
        double Kd{0.0};
        double Ts{0.01};
        double tau_f{0.005}; // Derivative filter time constant
        double u_min{-100.0};
        double u_max{100.0};
    };

    explicit PIDController(const Params& params) : params_(params) {
        reset();
    }

    void reset() {
        integrator_ = 0.0;
        prev_error_ = 0.0;
        filtered_derivative_ = 0.0;
    }

    double compute(double setpoint, double process_variable) {
        const double error = setpoint - process_variable;

        // Proportional term
        const double P = params_.Kp * error;

        // Derivative term with first-order filter
        const double alpha = params_.tau_f / (params_.tau_f + params_.Ts);
        const double raw_derivative = (error - prev_error_) / params_.Ts;
        filtered_derivative_ = alpha * filtered_derivative_ + (1.0 - alpha) * params_.Kd * raw_derivative;

        // Unclamped control signal using previous integrator state
        const double u_unclamped = P + integrator_ + filtered_derivative_;

        // Clamping & Anti-Windup Update
        double u_clamped = std::clamp(u_unclamped, params_.u_min, params_.u_max);

        // Conditional Integration: only integrate if not saturating in the direction of error
        bool saturating_high = (u_unclamped > params_.u_max) && (error > 0.0);
        bool saturating_low = (u_unclamped < params_.u_min) && (error < 0.0);

        if (!saturating_high && !saturating_low) {
            integrator_ += params_.Ki * params_.Ts * error;
        }

        prev_error_ = error;
        return u_clamped;
    }

private:
    Params params_;
    double integrator_{0.0};
    double prev_error_{0.0};
    double filtered_derivative_{0.0};
};

} // namespace Automation

#endif // PID_CONTROLLER_HPP
