#include "state_space.hpp"
#include <iomanip>
#include <iostream>

int main() {
    constexpr double m = 2.0;
    constexpr double c = 0.5;
    constexpr double k = 10.0;

    using System = Automation::StateSpaceSystem<2, 1, 1>;

    System::MatrixA A = {{{0.0, 1.0}, {-k / m, -c / m}}};
    System::MatrixB B = {{{0.0}, {1.0 / m}}};
    System::MatrixC C = {{{1.0, 0.0}}};
    System::MatrixD D = {{{0.0}}};

    System sys(A, B, C, D);

    constexpr double dt = 0.01;
    constexpr int total_steps = 100;
    System::InputVector u = {10.0};

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