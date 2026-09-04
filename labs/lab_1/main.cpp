#include "state_space.hpp"
#include <iomanip>
#include <iostream>

int main() {
    // Parameters: m = 2.0 kg, c = 0.5 Ns/m, k = 10.0 N/m
    constexpr double m = 2.0;
    constexpr double c = 0.5;
    constexpr double k = 10.0;

    // State-Space Matrices
    Automation::StateSpaceSystem<2, 1, 1>::MatrixA A = {{{0.0, 1.0}, {-k / m, -c / m}}};
    Automation::StateSpaceSystem<2, 1, 1>::MatrixB B = {{{0.0}, {1.0 / m}}};
    Automation::StateSpaceSystem<2, 1, 1>::MatrixC C = {{{1.0, 0.0}}};
    Automation::StateSpaceSystem<2, 1, 1>::MatrixD D = {{{0.0}}};

    // Use braces {} instead of parentheses () to prevent C++ most vexing parse
    Automation::StateSpaceSystem<2, 1, 1> sys{A, B, C, D};

    constexpr double dt = 0.01;
    constexpr int total_steps = 100;
    Automation::StateSpaceSystem<2, 1, 1>::InputVector u = {10.0}; // 10 N step force

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Step,Time(s),Position(m),Velocity(m/s)\n";

    for (int step = 0; step <= total_steps; ++step) {
        double t = step * dt;
        const auto& state = sys.getState();
        std::cout << step << "," << t << "," << state[0] << "," << state[1] << "\n";
        sys.stepRK4(u, dt);
    }

    return 0;
}