#!/usr/bin/env python3
"""
Chapter 1 Lab: Spring-Mass-Damper Dynamic Simulation
Models: m * d2x/dt2 + c * dx/dt + k * x = u(t)
State Vector: x = [position, velocity]^T
"""

import sys
import numpy as np

class SpringMassDamper:
    def __init__(self, m: float, c: float, k: float):
        self.m = m
        self.c = c
        self.k = k
        # State-space matrices
        self.A = np.array([[0.0, 1.0],
                           [-k / m, -c / m]])
        self.B = np.array([[0.0],
                           [1.0 / m]])
        self.C = np.array([[1.0, 0.0]])
        self.D = np.array([[0.0]])

    def state_derivative(self, x: np.ndarray, u: float) -> np.ndarray:
        return self.A @ x + self.B * u

    def rk4_step(self, x: np.ndarray, u: float, dt: float) -> np.ndarray:
        k1 = self.state_derivative(x, u)
        k2 = self.state_derivative(x + 0.5 * dt * k1, u)
        k3 = self.state_derivative(x + 0.5 * dt * k2, u)
        k4 = self.state_derivative(x + dt * k3, u)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

def run_simulation(steps: int = 100, dt: float = 0.01) -> None:
    # Mass = 2.0 kg, Damping = 0.5 Ns/m, Stiffness = 10.0 N/m
    system = SpringMassDamper(m=2.0, c=0.5, k=10.0)
    x = np.array([[0.0], [0.0]])  # Initial state: pos=0, vel=0
    u = 10.0  # Constant step force input (10 N)

    print(f"Step,Time(s),Position(m),Velocity(m/s)")
    for step in range(steps + 1):
        t = step * dt
        print(f"{step},{t:.2f},{x[0, 0]:.6f},{x[1, 0]:.6f}")
        x = system.rk4_step(x, u, dt)

if __name__ == "__main__":
    run_simulation()