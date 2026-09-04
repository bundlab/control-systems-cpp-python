#ifndef STATE_SPACE_HPP
#define STATE_SPACE_HPP

#include <array>
#include <cstddef>

namespace Automation {

template <std::size_t States, std::size_t Inputs, std::size_t Outputs>
class StateSpaceSystem {
public:
    using StateVector = std::array<double, States>;
    using InputVector = std::array<double, Inputs>;
    using OutputVector = std::array<double, Outputs>;
    using MatrixA = std::array<std::array<double, States>, States>;
    using MatrixB = std::array<std::array<double, Inputs>, States>;
    using MatrixC = std::array<std::array<double, States>, Outputs>;
    using MatrixD = std::array<std::array<double, Inputs>, Outputs>;

    StateSpaceSystem(const MatrixA& A, const MatrixB& B, const MatrixC& C, const MatrixD& D)
        : A_(A), B_(B), C_(C), D_(D) {
        state_.fill(0.0);
    }

    void setState(const StateVector& x) { state_ = x; }
    [[nodiscard]] const StateVector& getState() const { return state_; }

    StateVector computeDerivative(const StateVector& x, const InputVector& u) const {
        StateVector dxdt{};
        for (std::size_t i = 0; i < States; ++i) {
            dxdt[i] = 0.0;
            for (std::size_t j = 0; j < States; ++j) {
                dxdt[i] += A_[i][j] * x[j];
            }
            for (std::size_t j = 0; j < Inputs; ++j) {
                dxdt[i] += B_[i][j] * u[j];
            }
        }
        return dxdt;
    }

    void stepRK4(const InputVector& u, double dt) {
        StateVector k1 = computeDerivative(state_, u);

        StateVector x_k2{};
        for (std::size_t i = 0; i < States; ++i) {
            x_k2[i] = state_[i] + 0.5 * dt * k1[i];
        }
        StateVector k2 = computeDerivative(x_k2, u);

        StateVector x_k3{};
        for (std::size_t i = 0; i < States; ++i) {
            x_k3[i] = state_[i] + 0.5 * dt * k2[i];
        }
        StateVector k3 = computeDerivative(x_k3, u);

        StateVector x_k4{};
        for (std::size_t i = 0; i < States; ++i) {
            x_k4[i] = state_[i] + dt * k3[i];
        }
        StateVector k4 = computeDerivative(x_k4, u);

        for (std::size_t i = 0; i < States; ++i) {
            state_[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }

    OutputVector computeOutput(const InputVector& u) const {
        OutputVector y{};
        for (std::size_t i = 0; i < Outputs; ++i) {
            y[i] = 0.0;
            for (std::size_t j = 0; j < States; ++j) {
                y[i] += C_[i][j] * state_[j];
            }
            for (std::size_t j = 0; j < Inputs; ++j) {
                y[i] += D_[i][j] * u[j];
            }
        }
        return y;
    }

private:
    MatrixA A_;
    MatrixB B_;
    MatrixC C_;
    MatrixD D_;
    StateVector state_;
};

} // namespace Automation

#endif // STATE_SPACE_HPP