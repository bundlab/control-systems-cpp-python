# Chapter 3: Classical PID Control Architecture & Real-Time Engineering

This chapter covers production-ready discrete Proportional-Integral-Derivative (PID) controllers. You will learn to implement derivative low-pass filtering, anti-windup integration clamping, bumpless manual-to-automatic transfer, and step response behavior under dynamic load disturbances.

---

## 🔬 Lab 3: Discrete PID Controller with Anti-Windup & Derivative Filtering

### Objectives

1. Implement the discrete parallel PID control law:

$$u[k] = K_p e[k] + I[k] + D[k]$$


2. Integrate a first-order low-pass filter on the derivative term to suppress high-frequency noise:

$$D[k] = \alpha D[k-1] + (1 - \alpha) K_d \frac{e[k] - e[k-1]}{T_s}, \quad \alpha = \frac{\tau_f}{\tau_f + T_s}$$


3. Implement conditional integration clamping (anti-windup) to prevent accumulator saturation during actuator saturation ($u_{min} \le u[k] \le u_{max}$).
4. Simulate dynamic external load disturbances and compare the C++ implementation against Python's NumPy/SciPy reference.

---


## 1. Discrete PID Control Formulation

The continuous-time PID algorithm is expressed as:

$$u(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

Converting to a discrete formulation with sample period $T_s$:

- **Proportional Term:** $P[k] = K_p e[k]$
- **Integral Term (Forward Euler):** $I[k] = I[k-1] + K_i T_s e[k]$
- **Derivative Term with First-Order Filter:** 
  $$D[k] = \frac{\tau_f}{\tau_f + T_s} D[k-1] + \frac{K_d}{\tau_f + T_s} (e[k] - e[k-1])$$

Where $\tau_f$ is the filter time constant ($\tau_f = \frac{1}{2\pi f_c}$).

## 2. Anti-Windup Clamping Logic

Actuator physical limits ($u_{min}, u_{max}$) cause integral windup if the controller output saturates while tracking large step changes or recovering from external loads.


```
   +----+      Raw Control    +---------------+   Saturated

```
```
e[k] ->|PID |---> u_unclamped --->| Clamping /    |--> Control u[k]
+----+                     | Limiter       |
^                        +---------------+
|                                |
+---- Anti-Windup Feedback ------+

```

Integrator update rule with conditional integration:
$$\text{If } u_{unclamped} > u_{max} \text{ and } e[k] > 0, \text{ freeze } I[k] = I[k-1]$$
$$\text{If } u_{unclamped} < u_{min} \text{ and } e[k] < 0, \text{ freeze } I[k] = I[k-1]$$


---

