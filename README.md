# Automation and Control Systems: Practical Engineering with C++ and Python

A practical, code-first guide to designing, simulating, and implementing production-grade control systems and modern industrial automation pipelines.

---

## 📖 Table of Contents

### Part I: Fundamentals & Modeling
- [**Chapter 1: Foundations of Automation and Dynamic Systems**](./chapters/chapter_1.md) | [*Lab 1 Code*](./labs/lab_1)
  - Continuous vs. discrete-time dynamics
  - Mathematical modeling of mechanical and electrical systems
  - 4th-order Runge-Kutta (RK4) continuous ODE solver implementation
  - *Lab 1:* Spring-Mass-Damper continuous dynamic engine with zero-dependency C++ matrix solver.

* **[Chapter 2: State-Space Representation & Transfer Functions](https://www.google.com/search?q=./chapter_2_lab)**
* Matrix representations, controllability, and observability
* Discrete state-space conversion ($Z$-transform and zero-order hold)
* *Lab 2:* Pole-zero analysis, bode plot generation, and discrete state step updates.



### Part II: Classical & Advanced Control Implementation

* **[Chapter 3: Classical PID Control Architecture](https://www.google.com/search?q=./chapter_3_lab)**
* Anti-windup clamping, derivative kick mitigation, and low-pass filtering
* Digital PID implementation for real-time control loops
* *Lab 3:* Real-time C++ PID driver with dynamic load disturbance rejection.


* **[Chapter 4: State Feedback & Pole Placement Design](https://www.google.com/search?q=./chapter_4_lab)**
* Full-state feedback controller design and Ackermann’s formula
* Linear-Quadratic Regulator (LQR) optimization
* *Lab 4:* Eigen-based LQR controller for inverted pendulum stabilization.


* **[Chapter 5: State Estimation & Observer Design](https://www.google.com/search?q=./chapter_5_lab)**
* Luenberger observers and sensor fusion strategies
* Extended Kalman Filters (EKF) for non-linear state estimation
* *Lab 5:* Header-only C++ EKF implementation for noisy IMU state estimation.



### Part III: Digital Signal Processing & System Identification

* **[Chapter 6: System Identification & Parameter Estimation](https://www.google.com/search?q=./chapter_6_lab)**
* PRBS signal stimulus and Recursive Least Squares (RLS) estimation
* Auto-Regressive Exogenous (ARX) dynamic modeling
* *Lab 6:* Online RLS estimator tracking unknown DC motor dynamics.


* **[Chapter 7: Digital Filtering & Signal Conditioning](https://www.google.com/search?q=./chapter_7_lab)**
* IIR/FIR filter synthesis (Butterworth, Chebychev)
* Fixed-point arithmetic for resource-constrained microcontrollers
* *Lab 7:* Low-latency C++ signal filtering engine.



### Part IV: Industrial Communication & Motion Control

* **[Chapter 8: Industrial Communication Protocols](https://www.google.com/search?q=./chapter_8_lab)**
* Modbus TCP/RTU frame encoding and SocketCAN driver interfacing
* OPC UA client integration for distributed SCADA systems
* *Lab 8:* Asynchronous SocketCAN and Modbus TCP message processor.


* **[Chapter 9: Trajectory Generation & Motion Control](https://www.google.com/search?q=./chapter_9_lab)**
* S-curve and trapezoidal velocity profiling engines
* Multi-axis joint and Cartesian space motion planning
* *Lab 9:* Real-time S-curve trajectory generator engine.



### Part V: Model Predictive Control & Safety Systems

* **[Chapter 10: Model Predictive Control (MPC)](https://www.google.com/search?q=./chapter_10_lab)**
* Receding horizon control under state and input constraints
* Quadratic Programming (QP) solver integration
* *Lab 10:* Real-time MPC trajectory tracker using `qpOASES` and `cvxpy`.


* **[Chapter 11: Fault Detection, Diagnostics, and Safety Protocols](https://www.google.com/search?q=./chapter_11_lab)**
* Residual generation and model-based anomaly detection
* Failsafe State Machines (FSM) adhering to ISO 13849 standards
* *Lab 11:* Production-grade C++ safety watchdog system.



### Part VI: Software Architecture & CI/CD Pipelines

* **[Chapter 12: Production-Grade Software Architecture & CI/CD](https://www.google.com/search?q=./chapter_12_lab)**
* Deterministic POSIX thread scheduling (`SCHED_FIFO`) and lock-free ring buffers
* Automated continuous integration using GitHub Actions
* *Lab 12:* Hardware-in-the-Loop (HIL) testing pipeline with mock hardware interfaces.



---

## 🛠 Tech Stack & Dependencies

| Toolchain | Technology | Minimum Version |
| --- | --- | --- |
| **C++ Compiler** | GCC / Clang / MSVC | C++20 standard support required |
| **Build System** | CMake | 3.20+ |
| **C++ Libraries** | Eigen3, `qpOASES`, `libmodbus` | Latest stable |
| **Python Runtime** | Python | 3.10+ |
| **Python Libraries** | `numpy`, `scipy`, `control`, `cvxpy`, `matplotlib` | Latest stable |

---

## 🚀 Quickstart

### Build C++ Executables

```bash
mkdir build && cd build
cmake ..
cmake --build . --config Release
ctest --output-on-failure

```

### Run Lab Simulations

```bash
# Execute Chapter 1 Lab Driver
chmod +x chapter_1_lab/run_lab.sh
./chapter_1_lab/run_lab.sh

```

---

