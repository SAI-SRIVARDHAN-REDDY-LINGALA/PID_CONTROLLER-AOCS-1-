import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# === Quadcopter Physical Parameters ===
m = 1.0      # Mass (kg)
g = 9.81     # Gravity (m/s^2)
u_max = 20   # Max thrust (N)
u_min = 0    # Min thrust (N) (no negative thrust)

# === PID Gains ===
Kp, Ki, Kd = 10, 2, 3  # Tuned manually

# === Simulation Parameters ===
dt = 0.01  # Time step (s)
T = 5      # Total simulation time (s)
time = np.arange(0, T, dt)
n = len(time)

# === PID Controller with Anti-Windup ===
def pid_control(error, prev_error, integral, Kp, Ki, Kd, dt, umin, umax):
    integral += error * dt
    derivative = (error - prev_error) / dt if prev_error is not None else 0
    u = Kp * error + Ki * integral + Kd * derivative
    
    # Actuator Saturation + Anti-Windup
    if u > umax:
        u = umax
        integral -= error * dt  # Prevent integral windup
    elif u < umin:
        u = umin
        integral -= error * dt  # Prevent integral windup
    
    return u, integral

# === Low-Pass Filter for Noisy Sensor Readings ===
def low_pass_filter(y, alpha=0.2):
    if not hasattr(low_pass_filter, 'prev_value'):
        low_pass_filter.prev_value = y
    filtered = alpha * y + (1 - alpha) * low_pass_filter.prev_value
    low_pass_filter.prev_value = filtered
    return filtered

# === Simulation Loop ===
z = 0       # Initial altitude (m)
vz = 0      # Initial velocity (m/s)
z_ref = 5   # Desired altitude (m)
prev_error = None
integral = 0.0

z_out = np.zeros(n)
u_out = np.zeros(n)

for i in range(n):
    # Sensor Noise
    noise = np.random.normal(0, 0.1)
    z_measured = z + noise
    z_filtered = low_pass_filter(z_measured)

    # PID Control
    error = z_ref - z_filtered
    u, integral = pid_control(error, prev_error, integral, Kp, Ki, Kd, dt, u_min, u_max)

    # External Disturbance (Wind Gusts)
    wind_force = np.sin(0.5 * time[i]) * 2  # Small wind disturbance

    # Quadcopter Dynamics (Newton's Second Law)
    acceleration = (u - m * g + wind_force) / m  # a = F/m
    vz += acceleration * dt  # Integrate velocity
    z += vz * dt  # Integrate position

    # Store Values for Plotting
    z_out[i] = z
    u_out[i] = u
    prev_error = error

# === Plot Results ===
plt.figure(figsize=(12, 6))

# Altitude Plot
plt.subplot(2, 1, 1)
plt.plot(time, z_out, label="Altitude (m)")
plt.axhline(y=z_ref, color='r', linestyle='--', label="Reference")
plt.title("Quadcopter Altitude Control with PID")
plt.ylabel("Altitude (m)")
plt.legend()
plt.grid()

# Control Effort Plot
plt.subplot(2, 1, 2)
plt.plot(time, u_out, 'g', label="Thrust (N)")
plt.axhline(y=u_max, color='r', linestyle='--', label="Max Thrust")
plt.axhline(y=u_min, color='b', linestyle='--', label="Min Thrust")
plt.title("Control Effort (Thrust Input)")
plt.xlabel("Time (s)")
plt.ylabel("Thrust (N)")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
