import numpy as np
import matplotlib.pyplot as plt
import control as ct
from scipy.optimize import minimize, differential_evolution

# Define the transfer function G(s) = 1 / (s^3 + 3s^2 + 5s + 1)
num = [1]
den = [1, 3, 5, 1]
plant = ct.TransferFunction(num, den)

# Time axis for simulation
time_range = np.linspace(0, 10, 1000)

def compute_pid_output(err, prev_err, integral_term, P, I, D, delta_t, threshold=0.01):
    if abs(err) < threshold:
        return 0, integral_term  # Dead zone handling
    integral_term += err * delta_t
    derivative_term = (err - prev_err) / delta_t if prev_err is not None else 0
    control_signal = P * err + I * integral_term + D * derivative_term
    return control_signal, integral_term

def smooth_signal(value, alpha=0.1):
    if not hasattr(smooth_signal, 'previous_value'):
        smooth_signal.previous_value = value
    smoothed = alpha * value + (1 - alpha) * smooth_signal.previous_value
    smooth_signal.previous_value = smoothed
    return smoothed

def evaluate_pid(params):
    P, I, D = params
    pid_tf = ct.TransferFunction([D, P, I], [1, 0])
    closed_loop_system = ct.feedback(pid_tf * plant)
    t_resp, y_resp = ct.step_response(closed_loop_system, time_range)
    overshoot = np.max(y_resp) - 1
    settling_idx = np.where(np.abs(y_resp - 1) < 0.01)[0]
    settling_time = t_resp[settling_idx[-1]] if settling_idx.size else 10
    steady_state_error = np.abs(y_resp[-1] - 1)
    noise_effect = np.var(np.diff(y_resp))
    control_usage = np.sum(np.abs(np.diff(y_resp)))
    return (100 * overshoot) + (5 * settling_time) + (100 * steady_state_error) + (0.1 * control_usage) + (50 * noise_effect)

# Define tuning bounds
pid_bounds = [(0.1, 100), (0.1, 100), (0.1, 3)]

# Optimization using Nelder-Mead
nm_result = minimize(evaluate_pid, [0.1, 0.01, 0.01], bounds=pid_bounds, method='Nelder-Mead')
nm_P, nm_I, nm_D = nm_result.x

# Optimization using Genetic Algorithm
ga_result = differential_evolution(evaluate_pid, pid_bounds, strategy='best1bin', maxiter=500, popsize=20, tol=0.01)
ga_P, ga_I, ga_D = ga_result.x

# Selecting best PID parameters
cost_values = {
    'Nelder-Mead': evaluate_pid([nm_P, nm_I, nm_D]),
    'Genetic Algorithm': evaluate_pid([ga_P, ga_I, ga_D])
}
best_approach = min(cost_values, key=cost_values.get)
P_best, I_best, D_best = (nm_P, nm_I, nm_D) if best_approach == 'Nelder-Mead' else (ga_P, ga_I, ga_D)

# Root Locus Plot
plt.figure()
ct.root_locus(plant)
plt.title("Root Locus of Open-Loop System")
plt.show()

# Bode Plot
plt.figure()
ct.bode(plant, dB=True)
plt.title("Bode Plot of Open-Loop System")
plt.show()

# Simulate optimized system
optimal_pid = ct.TransferFunction([D_best, P_best, I_best], [1, 0])
optimized_system = ct.feedback(optimal_pid * plant)
t_sim, y_sim = ct.step_response(optimized_system, time_range)

# Plot results
plt.figure(figsize=(10, 5))
plt.plot(t_sim, y_sim, label="Optimized PID Response")
plt.axhline(1, color="r", linestyle="--", label="Reference")
plt.xlabel("Time (s)")
plt.ylabel("Output")
plt.title("Optimized PID Step Response")
plt.legend()
plt.grid()
plt.show()

# State-space representation
A_matrix = np.array([[0, 1, 0], [0, 0, 1], [-1, -5, -3]])
B_matrix = np.array([0, 0, 1])
C_matrix = np.array([1, 0, 0])
D_matrix = 0

# Simulation parameters for dynamic response
time_step = 0.005
t_points = np.arange(0, 5, time_step)
data_length = len(t_points)

# Initialize states and control variables
state = np.array([0.0, 0.0, 0.0])
int_term = 0.0
prev_err = None
control_min, control_max = -20, 20
measurement_noise = 0.05

# Storage for output and control
output_values = np.zeros(data_length)
control_values = np.zeros(data_length)
reference_signal = np.ones(data_length)

for i in range(data_length):
    ref_val = reference_signal[i]
    measured_output = state[0] + np.random.normal(0, measurement_noise)
    filtered_output = smooth_signal(measured_output)
    err_signal = ref_val - filtered_output
    control_input, int_term = compute_pid_output(err_signal, prev_err, int_term, P_best, I_best, D_best, time_step, threshold=0.01)
    control_limited = np.clip(control_input, control_min, control_max)
    control_values[i] = control_limited
    
    state_derivative = np.array([state[1], state[2], -state[0] - 5*state[1] - 3*state[2] + control_limited])
    state += state_derivative * time_step
    output_values[i] = state[0]
    prev_err = err_signal

# Plot system response
plt.figure(figsize=(12, 8))
plt.subplot(2,1,1)
plt.plot(t_points, output_values, label='System Output')
plt.plot(t_points, reference_signal, 'r--', label='Reference')
plt.title('System Response')
plt.ylabel('Output')
plt.legend()

plt.subplot(2,1,2)
plt.plot(t_points, control_values, 'g', label='Control Input')
plt.title('Control Input Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Control Signal')
plt.legend()
plt.tight_layout()
plt.show()
