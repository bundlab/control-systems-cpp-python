# Chapter: Foundations of Automation and Dynamic Systems

This document provides a complete technical explanation of the theory, mathematical derivations, software architecture, and execution details for **Chapter 1 Lab: Simulating Spring-Mass-Damper Dynamics**.

---

## 1. Dynamic System Modeling

Physical systems in control engineering are represented using continuous differential equations derived from first principles (Newton’s laws of motion, Kirchhoff’s circuit laws, etc.).

A standard mechanical **spring-mass-damper** system driven by an external force $u(t)$ is governed by Newton's second law:

$$m \frac{d^2 x(t)}{dt^2} + c \frac{dx(t)}{dt} + k x(t) = u(t)$$

Where:

* $m$ = Mass of the object ($\text{kg}$)
* $c$ = Damping coefficient ($\text{N}\cdot\text{s/m}$)
* $k$ = Spring stiffness coefficient ($\text{N/m}$)
* $x(t)$ = Displacement/Position ($\text{m}$)
* $u(t)$ = External applied force ($\text{N}$)

---

## 2. State-Space Form Derivation

To compute dynamic updates programmatically, higher-order differential equations are converted into a system of coupled first-order differential equations using **State-Space Representation**.

### Step 1: Define State Variables

Let the state vector $\mathbf{x}(t)$ consist of position $x_1(t)$ and velocity $x_2(t)$:

$$x_1(t) = x(t)$$

$$x_2(t) = \dot{x}(t) = \frac{dx(t)}{dt}$$

### Step 2: Formulate State Derivatives

Taking the time derivative of each state variable yields:

$$\dot{x}_1(t) = x_2(t)$$

$$\dot{x}_2(t) = \ddot{x}(t) = -\frac{k}{m} x_1(t) - \frac{c}{m} x_2(t) + \frac{1}{m} u(t)$$

### Step 3: Matrix Standard Form

Expressing the coupled equations in standard matrix form:

$$\mathbf{\dot{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B}\mathbf{u}(t)$$

$$\mathbf{y}(t) = \mathbf{C}\mathbf{x}(t) + \mathbf{D}\mathbf{u}(t)$$

$$\begin{bmatrix} \dot{x}_1(t) \\ \dot{x}_2(t) \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ -\frac{k}{m} & -\frac{c}{m} \end{bmatrix} \begin{bmatrix} x_1(t) \\ x_2(t) \end{bmatrix} + \begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix} u(t)$$

$$\mathbf{y}(t) = \begin{bmatrix} 1 & 0 \end{bmatrix} \begin{bmatrix} x_1(t) \\ x_2(t) \end{bmatrix} + \begin{bmatrix} 0 \end{bmatrix} u(t)$$

---

## 3. Numerical Integration (Explicit RK4)

Since computers process discrete time steps $\Delta t$, continuous dynamics must be integrated numerically. Simple Euler integration ($x_{k+1} = x_k + \Delta t \cdot \dot{x}_k$) accumulates truncation error quickly. We utilize the **4th-Order Runge-Kutta (RK4)** integration scheme, which evaluates four intermediate derivative samples across the time interval $\Delta t$:

Given $\dot{\mathbf{x}} = f(t, \mathbf{x}, u)$:

$$\mathbf{k}_1 = f(t_n, \mathbf{x}_n, u_n)$$

$$\mathbf{k}_2 = f\left(t_n + \frac{\Delta t}{2}, \mathbf{x}_n + \frac{\Delta t}{2}\mathbf{k}_1, u_n\right)$$

$$\mathbf{k}_3 = f\left(t_n + \frac{\Delta t}{2}, \mathbf{x}_n + \frac{\Delta t}{2}\mathbf{k}_2, u_n\right)$$

$$\mathbf{k}_4 = f(t_n + \Delta t, \mathbf{x}_n + \Delta t \mathbf{k}_3, u_n)$$

$$\mathbf{x}_{n+1} = \mathbf{x}_n + \frac{\Delta t}{6} \left(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4\right)$$

---

## 4. Implementation Details

### Python Implementation (`simulate.py`)

* **Role:** Prototyping and quick analysis tool using `numpy`.
* **Logic:** Defines `SpringMassDamper` class holding continuous matrices $\mathbf{A}, \mathbf{B}, \mathbf{C}, \mathbf{D}$. Uses matrix multiplication (`@`) inside a dynamic `rk4_step()` routine to step state vectors forward. Output is formatted as CSV to stdout.

### C++ Implementation (`state_space.hpp` & `main.cpp`)

* **Role:** Production-ready real-time engine template.
* **Design:** Zero-dependency template class `Automation::StateSpaceSystem<States, Inputs, Outputs>` utilizing modern C++20 standard array containers (`std::array`) for heap-allocation-free compile-time optimizations.
* **RK4 Integration Engine:** Computes intermediate vectors $\mathbf{k}_1$ through $\mathbf{k}_4$ explicitly without external linear algebra dependencies, maintaining maximum portability across microcontrollers and Linux platforms.

---

## 5. Running the Simulation

Execute the automated shell script to compile the native C++ binary via CMake, execute both Python and C++ engines, and verify output parity:

```bash
chmod +x chapter_1_lab/run_lab.sh
./chapter_1_lab/run_lab.sh

```

### Expected Output Summary

When given parameters $m=2.0\,\text{kg}$, $c=0.5\,\text{Ns/m}$, $k=10.0\,\text{N/m}$, and step force input $u=10.0\,\text{N}$, the system exhibits an underdamped second-order step response stabilizing around final position $x_{\text{final}} = \frac{u}{k} = \frac{10.0}{10.0} = 1.0\,\text{m}$.