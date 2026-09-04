#ifndef DISCRETE_SYSTEM_HPP
#define DISCRETE_SYSTEM_HPP

#include <array>
#include <cstddef>
#include <cmath>

namespace Automation {

template <std::size_t States, std::size_t Inputs, std::size_t Outputs>
class DiscreteStateSpace {
public:
    using StateVector = std::array<double, States>;
    using InputVector = std::array<double, Inputs>;
    using OutputVector = std::array<double, Outputs>;
    using MatrixA = std::array<std::array<double, States>, States>;
    using MatrixB = std::array<std::array<double, Inputs>, States>;
    using MatrixC = std::array<std::array<double, States>, Outputs>;
    using MatrixD = std::array<std::array<double, Inputs>, Outputs>;

    DiscreteStateSpace(const MatrixA& Ad, const MatrixB& Bd, const MatrixC& C, const MatrixD& D)
        : Ad_(Ad), Bd_(Bd), C_(C), D_(D) {
        x_.fill(0.0);
    }

    void setState(const StateVector& x) { x_ = x; }
    [[nodiscard]] const StateVector& getState() const { return x_; }

    // Discrete State Update: x[k+1] = Ad * x[k] + Bd * u[k]
    void update(const InputVector& u) {
        StateVector x_next{};
        for (std::size_t i = 0; i < States; ++i) {
            x_next[i] = 0.0;
            for (std::size_t j = 0; j < States; ++j) {
                x_next[i] += Ad_[i][j] * x_[j];
            }
            for (std::size_t j = 0; j < Inputs; ++j) {
                x_next[i] += Bd_[i][j] * u[j];
            }
        }
        x_ = x_next;
    }

    // Output Equation: y[k] = C * x[k] + D * u[k]
    OutputVector computeOutput(const InputVector& u) const {
        OutputVector y{};
        for (std::size_t i = 0; i < Outputs; ++i) {
            y[i] = 0.0;
            for (std::size_t j = 0; j < States; ++j) {
                y[i] += C_[i][j] * x_[j];
            }
            for (std::size_t j = 0; j < Inputs; ++j) {
                y[i] += D_[i][j] * u[j];
            }
        }
        return y;
    }

    // ZOH Taylor-Series Discretization Helper for 2x2 Continuous Systems
    static void discretizeZOH(const MatrixA& A, const MatrixB& B, double Ts, MatrixA& Ad, MatrixB& Bd) {
        // Compute matrix exponential e^(A*Ts) via Taylor expansion (up to 10 terms)
        MatrixA term{};
        for (std::size_t i = 0; i < States; ++i) {
            for (std::size_t j = 0; j < States; ++j) {
                Ad[i][j] = (i == j) ? 1.0 : 0.0; // Identity matrix
                term[i][j] = Ad[i][j];
            }
        }

        double factorial = 1.0;
        for (int k = 1; k <= 12; ++k) {
            factorial *= k;
            MatrixA next_term{};
            for (std::size_t i = 0; i < States; ++i) {
                for (std::size_t j = 0; j < States; ++j) {
                    next_term[i][j] = 0.0;
                    for (std::size_t m = 0; m < States; ++m) {
                        next_term[i][j] += term[i][m] * A[m][j] * Ts;
                    }
                }
            }
            term = next_term;
            for (std::size_t i = 0; i < States; ++i) {
                for (std::size_t j = 0; j < States; ++j) {
                    Ad[i][j] += term[i][j] / factorial;
                }
            }
        }

        // Bd integral expansion: Bd = sum_{k=0}^N (A^k * Ts^(k+1) / (k+1)!) * B
        MatrixA integral{};
        for (std::size_t i = 0; i < States; ++i) {
            for (std::size_t j = 0; j < States; ++j) {
                integral[i][j] = (i == j) ? Ts : 0.0;
            }
        }

        MatrixA term_A = A;
        double fact_k = 1.0;
        double pow_Ts = Ts;
        for (int k = 1; k <= 12; ++k) {
            pow_Ts *= Ts;
            fact_k *= (k + 1);
            for (std::size_t i = 0; i < States; ++i) {
                for (std::size_t j = 0; j < States; ++j) {
                    integral[i][j] += term_A[i][j] * pow_Ts / fact_k;
                }
            }
            // term_A = term_A * A
            MatrixA next_A{};
            for (std::size_t i = 0; i < States; ++i) {
                for (std::size_t j = 0; j < States; ++j) {
                    next_A[i][j] = 0.0;
                    for (std::size_t m = 0; m < States; ++m) {
                        next_A[i][j] += term_A[i][m] * A[m][j];
                    }
                }
            }
            term_A = next_A;
        }

        // Bd = integral * B
        for (std::size_t i = 0; i < States; ++i) {
            for (std::size_t j = 0; j < Inputs; ++j) {
                Bd[i][j] = 0.0;
                for (std::size_t m = 0; m < States; ++m) {
                    Bd[i][j] += integral[i][m] * B[m][j];
                }
            }
        }
    }

private:
    MatrixA Ad_;
    MatrixB Bd_;
    MatrixC C_;
    MatrixD D_;
    StateVector x_;
};

} // namespace Automation

#endif // DISCRETE_SYSTEM_HPP
