import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution  
import control as ctrl


# Define system transfer function G(s) = 1 / (s^3 + 3s^2 + 5s + 1)
sys_num = [1]
sys_den = [1, 3, 5, 1]
system = ctrl.TransferFunction(sys_num, sys_den)

# Simulation time range
time_span = np.linspace(0, 10, 1000)

# Objective function for PID tuning
def evaluate_pid(parameters):
    P, I, D = parameters
    pid_controller = ctrl.TransferFunction([D, P, I], [1, 0])
    closed_loop_system = ctrl.feedback(pid_controller * system)
    t, response = ctrl.step_response(closed_loop_system, time_span)
    
    peak_overshoot = np.max(response) - 1
    final_value_error = np.abs(response[-1] - 1)
    settling_idx = np.where(np.abs(response - 1) < 0.01)[0]
    settling_time = t[settling_idx[-1]] if len(settling_idx) > 0 else 10
    noise_variation = np.var(np.diff(response))
    control_effort = np.sum(np.abs(np.diff(response)))
    
    return (100 * peak_overshoot) + (5 * settling_time) + (100 * final_value_error) + (0.1 * control_effort) + (50 * noise_variation)

# Optimization bounds
param_bounds = [(0.1, 100), (0.1, 100), (0.1, 3)]

# Perform optimization using differential evolution
optimized_result = differential_evolution(evaluate_pid, param_bounds, strategy='best1bin', maxiter=500, popsize=20, tol=0.01)
optimal_Kp, optimal_Ki, optimal_Kd = optimized_result.x

# Construct PID variations
I_PD_control = ctrl.TransferFunction([optimal_Ki], [1, 0])
PI_D_control = ctrl.TransferFunction([optimal_Kp, optimal_Ki], [1, 0]) + ctrl.TransferFunction([optimal_Kd, 0], [1])
PID_control = ctrl.TransferFunction([optimal_Kd, optimal_Kp, optimal_Ki], [1, 0])

# Closed-loop systems
pid_response = ctrl.feedback(PID_control * system)
ipd_response = ctrl.feedback(I_PD_control * system)
pid_mod_response = ctrl.feedback(PI_D_control * system)

# Step responses
pid_time, pid_output = ctrl.step_response(pid_response, time_span)
ipd_time, ipd_output = ctrl.step_response(ipd_response, time_span)
pid_mod_time, pid_mod_output = ctrl.step_response(pid_mod_response, time_span)

# Plot step responses
plt.figure(figsize=(10, 6))
plt.plot(pid_time, pid_output, label="PID Response")
plt.plot(ipd_time, ipd_output, label="I-PD Response")
plt.plot(pid_mod_time, pid_mod_output, label="PI-D Response")
plt.axhline(1, color="r", linestyle="--", label="Reference")
plt.xlabel("Time (s)")
plt.ylabel("System Output")
plt.title("Comparison of PID, I-PD, and PI-D Controllers")
plt.legend()
plt.grid()
plt.show()

