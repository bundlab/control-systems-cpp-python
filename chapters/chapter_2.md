# Chapter 2: State-Space Representation & Transfer Functions

This chapter covers converting classical Transfer Functions $G(s)$ into continuous state-space models, computing Controllability and Observability matrices, and discretizing continuous dynamics into discrete-time state-space models using Zero-Order Hold (ZOH).

---

## 🔬 Lab 2: Transfer Function Conversion, Controllability, & ZOH Discretization

### Objectives

1. Convert a second-order continuous Transfer Function:

$$G(s) = \frac{Y(s)}{U(s)} = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$



into **Controllable Canonical Form** state-space representation.
2. Evaluate System **Controllability** ($\mathbf{\mathcal{C}}$) and **Observability** ($\mathbf{\mathcal{O}}$) matrices by computing their full-rank conditions:

$$\mathbf{\mathcal{C}} = \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} \end{bmatrix}, \quad \mathbf{\mathcal{O}} = \begin{bmatrix} \mathbf{C} \\ \mathbf{C}\mathbf{A} \end{bmatrix}$$


3. Discretize continuous matrices $(\mathbf{A}, \mathbf{B})$ to discrete matrices $(\mathbf{A}_d, \mathbf{B}_d)$ using Zero-Order Hold (ZOH) matrix exponential expansion:

$$\mathbf{A}_d = e^{\mathbf{A} T_s} = \mathbf{I} + \mathbf{A} T_s + \frac{\mathbf{A}^2 T_s^2}{2!} + \dots$$


$$\mathbf{B}_d = \left( \int_0^{T_s} e^{\mathbf{A} \tau} d\tau \right) \mathbf{B} = \mathbf{A}^{-1} (\mathbf{A}_d - \mathbf{I}) \mathbf{B}$$


4. Implement a discrete C++20 State-Space system simulator and compare state updates directly against Python's `control` library.

---


## 1. Controllable Canonical Form

Given an $n$-th order continuous transfer function:

$$G(s) = \frac{b_{n-1} s^{n-1} + \dots + b_1 s + b_0}{s^n + a_{n-1} s^{n-1} + \dots + a_1 s + a_0}$$

The **Controllable Canonical Form** defines state variables such that inputs directly drive state transitions:

$$\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \\ \vdots \\ \dot{x}_n \end{bmatrix} = \begin{bmatrix} 0 & 1 & 0 & \dots & 0 \\ 0 & 0 & 1 & \dots & 0 \\ \vdots & \vdots & \vdots & \ddots & \vdots \\ -a_0 & -a_1 & -a_2 & \dots & -a_{n-1} \end{bmatrix} \begin{bmatrix} x_1 \\ x_2 \\ \vdots \\ x_n \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ \vdots \\ 1 \end{bmatrix} u(t)$$

$$\mathbf{y}(t) = \begin{bmatrix} b_0 & b_1 & \dots & b_{n-1} \end{bmatrix} \mathbf{x}(t)$$

## 2. Controllability & Observability

- **Controllability:** A system is controllable if an unconstrained control input $u(t)$ can steer any initial state $\mathbf{x}(0)$ to any target state $\mathbf{x}(t_f)$ within finite time.
  $$\text{rank}(\mathbf{\mathcal{C}}) = \text{rank}\left( \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} & \dots & \mathbf{A}^{n-1}\mathbf{B} \end{bmatrix} \right) = n$$

- **Observability:** A system is observable if the initial state $\mathbf{x}(0)$ can be uniquely determined from knowledge of the output $\mathbf{y}(t)$ and input $\mathbf{u}(t)$ over a finite interval.
  $$\text{rank}(\mathbf{\mathcal{O}}) = \text{rank}\left( \begin{bmatrix} \mathbf{C} \\ \mathbf{C}\mathbf{A} \\ \vdots \\ \mathbf{C}\mathbf{A}^{n-1} \end{bmatrix} \right) = n$$

## 3. Exact Zero-Order Hold (ZOH) Discretization

In real-time digital controllers (microcontrollers, PLCs), states update at fixed sampling periods $T_s$:

$$\mathbf{x}[k+1] = \mathbf{A}_d \mathbf{x}[k] + \mathbf{B}_d \mathbf{u}[k]$$

Where:
$$\mathbf{A}_d = e^{\mathbf{A} T_s}$$
$$\mathbf{B}_d = \mathbf{A}^{-1} (\mathbf{A}_d - \mathbf{I}) \mathbf{B}$$