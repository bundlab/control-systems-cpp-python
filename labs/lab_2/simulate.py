#!/usr/bin/env python3
"""
Lab 2 Python Verification: Discretization & State-Space Updates
"""

import numpy as np
import scipy.signal as signal
import scipy.linalg as linalg

def main():
    wn = 5.0
    zeta = 0.6
    Ts = 0.01

    # Continuous State-Space (Controllable Canonical Form)
    A = np.array([[0.0, 1.0], [-wn**2, -2.0 * zeta * wn]])
    B = np.array([[0.0], [wn**2]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    # Check Controllability & Observability
    Ctrbg = np.hstack((B, A @ B))
    Obsvg = np.vstack((C, C @ A))

    _ = np.linalg.det(Ctrbg)
    _ = np.linalg.det(Obsvg)

    # Discretize using ZOH ('zoh')
    sys_cont = signal.StateSpace(A, B, C, D)
    sys_disc = sys_cont.to_discrete(dt=Ts, method='zoh')

    Ad = sys_disc.A
    Bd = sys_disc.B

    x = np.array([[0.0], [0.0]])
    u = np.array([[1.0]])

    print("Step,Time(s),Output(y),State_x1,State_x2")
    for step in range(101):
        t = step * Ts
        y = C @ x + D @ u
        print(f"{step},{t:.2f},{y[0,0]:.6f},{x[0,0]:.6f},{x[1,0]:.6f}")
        x = Ad @ x + Bd @ u

if __name__ == "__main__":
    main()