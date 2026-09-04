#!/usr/bin/env python3
"""
Lab 3 Python Verification: Discrete PID Controller with Anti-Windup & Load Disturbance
"""

import numpy as np

class PIDController:
    def __init__(self, Kp, Ki, Kd, Ts, tau_f, u_min, u_max):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.alpha = tau_f / (tau_f + Ts)
        self.u_min = u_min
        self.u_max = u_max

        self.integrator = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0

    def compute(self, setpoint, process_variable):
        error = setpoint - process_variable
        P = self.Kp * error

        raw_derivative = (error - self.prev_error) / self.Ts
        self.filtered_derivative = self.alpha * self.filtered_derivative + (1.0 - self.alpha) * self.Kd * raw_derivative

        u_unclamped = P + self.integrator + self.filtered_derivative
        u_clamped = np.clip(u_unclamped, self.u_min, self.u_max)

        saturating_high = (u_unclamped > self.u_max) and (error > 0.0)
        saturating_low = (u_unclamped < self.u_min) and (error < 0.0)

        if not saturating_high and not saturating_low:
            self.integrator += self.Ki * self.Ts * error

        self.prev_error = error
        return u_clamped

def main():
    dt = 0.01
    steps = 500
    m, c, k = 2.0, 0.5, 10.0

    A = np.array([[0.0, 1.0], [-k/m, -c/m]])
    B = np.array([[0.0], [1.0/m]])

    pid = PIDController(Kp=120.0, Ki=80.0, Kd=15.0, Ts=dt, tau_f=0.005, u_min=-30.0, u_max=30.0)

    x = np.array([[0.0], [0.0]])
    setpoint = 1.0

    print("Step,Time(s),Setpoint,Position(m),Velocity(m/s),Control(N),Disturbance(N)")

    for step in range(steps + 1):
        t = step * dt
        disturbance = -15.0 if t >= 2.5 else 0.0
        
        u_control = pid.compute(setpoint, x[0, 0])
        u_total = u_control + disturbance

        print(f"{step},{t:.2f},{setpoint:.2f},{x[0,0]:.6f},{x[1,0]:.6f},{u_control:.6f},{disturbance:.2f}")

        # RK4 Step Integration
        def deriv(x_state, u_val):
            return A @ x_state + B * u_val

        k1 = deriv(x, u_total)
        k2 = deriv(x + 0.5 * dt * k1, u_total)
        k3 = deriv(x + 0.5 * dt * k2, u_total)
        k4 = deriv(x + dt * k3, u_total)

        x += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

if __name__ == "__main__":
    main()
