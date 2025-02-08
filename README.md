# PID_CONTROLLER-AOCS-1-
PID Controller for a Higher-Order System

PID Controller Design for a Higher-Order System with Real-World Constraints
Overview
This project implements a PID controller for a complex third-order system while considering real-world constraints such as:

Actuator saturation (limits on control effort)
Sensor noise (random inaccuracies in measurements)
Noise filtering using a low-pass filter
The goal is to analyze PID control trade-offs in challenging environments and optimize performance.

System Model

We consider a higher-order system with the transfer function:G(s) 

 
A PID controller is designed and tuned using optimization techniques (Nelder-Mead & Genetic Algorithm) to achieve the best performance.

Features
✔️ Real-world constraints (actuator limits, sensor noise, and filtering)
✔️ Step response analysis (stability, overshoot, settling time)
✔️ Root locus & Bode plot analysis
✔️ PID optimization using numerical methods
✔️ State-space simulation

Implementation
The Python implementation includes:

Transfer function modeling using control library
PID tuning using scipy.optimize
Noise simulation to mimic sensor inaccuracies
Actuator constraints for realistic control effort
Visualization of system response


Results & Analysis

📌 Trade-offs in PID control:
High Kp: Faster response but higher overshoot
High Ki: Reduces steady-state error but can cause instability
High Kd: Improves damping but increases control effort
