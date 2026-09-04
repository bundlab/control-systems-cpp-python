#include "discrete_system.hpp"
#include <iomanip>
#include <iostream>

int main() {
    // 2nd Order System: wn = 5.0 rad/s, zeta = 0.6
    constexpr double wn = 5.0;
    constexpr double zeta = 0.6;
    constexpr double Ts = 0.01; // Sampling period = 10 ms

    using System = Automation::DiscreteStateSpace<2, 1, 1>;

    // Continuous Controllable Canonical Matrices
    System::MatrixA A = {{{0.0, 1.0}, {-wn * wn, -2.0 * zeta * wn}}};
    System::MatrixB B = {{{0.0}, {wn * wn}}};
    System::MatrixC C = {{{1.0, 0.0}}};
    System::MatrixD D = {{{0.0}}};

    System::MatrixA Ad{};
    System::MatrixB Bd{};

    // Compute ZOH Discretization
    System::discretizeZOH(A, B, Ts, Ad, Bd);

    System sys(Ad, Bd, C, D);
    System::InputVector u = {1.0}; // Unit step input

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Step,Time(s),Output(y),State_x1,State_x2\n";

    constexpr int total_steps = 100;
    for (int step = 0; step <= total_steps; ++step) {
        double t = step * Ts;
        const auto& state = sys.getState();
        auto y = sys.computeOutput(u);
        std::cout << step << "," << t << "," << y[0] << "," << state[0] << "," << state[1] << "\n";
        sys.update(u);
    }

    return 0;
}
