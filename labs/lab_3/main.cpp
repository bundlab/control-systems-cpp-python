#include "pid_controller.hpp"
#include "../lab_1/state_space.hpp"
#include <iomanip>
#include <iostream>

int main() {
    constexpr double dt = 0.01;
    constexpr int total_steps = 500; // 5.0 seconds total

    // Plant Setup: Mass-Spring-Damper (m=2.0, c=0.5, k=10.0)
    constexpr double m = 2.0, c = 0.5, k = 10.0;
    using Plant = Automation::StateSpaceSystem<2, 1, 1>;

    Plant::MatrixA A = {{{0.0, 1.0}, {-k / m, -c / m}}};
    Plant::MatrixB B = {{{0.0}, {1.0 / m}}};
    Plant::MatrixC C = {{{1.0, 0.0}}};
    Plant::MatrixD D = {{{0.0}}};

    Plant plant(A, B, C, D);

    // PID Configuration
    Automation::PIDController::Params pid_params;
    pid_params.Kp = 120.0;
    pid_params.Ki = 80.0;
    pid_params.Kd = 15.0;
    pid_params.Ts = dt;
    pid_params.tau_f = 0.005;
    pid_params.u_min = -30.0; // Actuator limits
    pid_params.u_max = 30.0;

    Automation::PIDController pid(pid_params);

    double setpoint = 1.0; // 1 meter position setpoint

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Step,Time(s),Setpoint,Position(m),Velocity(m/s),Control(N),Disturbance(N)\n";

    for (int step = 0; step <= total_steps; ++step) {
        double t = step * dt;

        // Dynamic Load Disturbance: -15 N step load applied at t = 2.5s
        double disturbance = (t >= 2.5) ? -15.0 : 0.0;

        const auto& state = plant.getState();
        double current_position = state[0];

        // Compute PID Control Action
        double u_control = pid.compute(setpoint, current_position);

        // Apply total input = control + load disturbance
        Plant::InputVector total_input = {u_control + disturbance};

        std::cout << step << "," << t << "," << setpoint << "," 
                  << state[0] << "," << state[1] << "," 
                  << u_control << "," << disturbance << "\n";

        plant.stepRK4(total_input, dt);
    }

    return 0;
}
