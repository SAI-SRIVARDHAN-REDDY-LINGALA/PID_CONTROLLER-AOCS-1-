# PID Controller for a Higher-Order System (AOCS-1)   FOR STUDENT SATELLITE PROGRAM IIT KHARAGPUR 

Got it! Here's an updated `README.md` that includes both files (`PID_3rdOrder_controller.py` and `PID_TUNNING.py`) for better organization.  

---

# **PID Controller for a Higher-Order System (AOCS-1)**  

## **Introduction**  
This project implements a **PID controller** for a **third-order system**, incorporating real-world constraints such as:  

- **Actuator Saturation:** Limits on control effort to prevent excessive input.  
- **Sensor Noise:** Random inaccuracies in measurements to simulate real-world conditions.  
- **Noise Filtering:** A **low-pass filter** is used to smooth noisy sensor readings.  

The objective is to analyze **PID control trade-offs** in complex environments and optimize system performance using numerical methods.  

---

## **System Model**  

The system under consideration has the **third-order transfer function**:  

G(s) = 1/{s^3 + 3s^2 + 5s + 1}

A **PID controller** is designed and optimized using:  
1. **Nelder-Mead Optimization** – A derivative-free approach for refining PID gains.  
2. **Genetic Algorithm (GA)** – An evolutionary approach to find globally optimal PID parameters.  

---

## **Project Structure 📂**  

```
📦 PID_CONTROLLER-AOCS-1  
│── 📜 README.md                # Project documentation  
│── 📜 LICENSE                  # License file  
│── 📜 PID_3rdOrder_controller.py  # Implements PID control on the system  
│── 📜 PID_TUNNING.py            # Optimizes PID parameters using GA and Nelder-Mead  
│── 📂 results                   # Folder for simulation results (plots, logs)  
│── 📂 docs                      # Additional project documentation  
```

---

## **Features 🚀**  

✔️ **Real-world constraints:** Actuator limits, sensor noise, and filtering.  
✔️ **Step response analysis:** Evaluating stability, overshoot, and settling time.  
✔️ **Root locus & Bode plot analysis:** Understanding system dynamics.  
✔️ **PID tuning via numerical methods:** Nelder-Mead & Genetic Algorithm.  
✔️ **State-space simulation:** Advanced system modeling and analysis.  

---

## **Installation 📦**  

Ensure you have Python installed and install the required dependencies using:  

```bash
pip install numpy matplotlib control scipy
```

---

## **Usage 🔧**  

### **1️⃣ Run the PID Controller Simulation**  

```bash
python PID_3rdOrder_controller.py
```
- Simulates the PID response for a **third-order system**.  
- Implements **actuator saturation, sensor noise, and filtering**.  
- Plots **step response, root locus, and Bode plot**.  

### **2️⃣ Run the PID Tuning Optimization**  

```bash
python PID_TUNNING.py
```
- Uses **Nelder-Mead** and **Genetic Algorithm** to optimize PID parameters.  
- Compares different PID variants (**PID, PI-D, I-PD**).  
- Saves optimized values for further use.  

---

## **Results & Analysis 📊**  

### 📌 **PID Trade-offs:**  
- **High Kp** → Faster response but higher overshoot.  
- **High Ki** → Reduces steady-state error but may cause instability.  
- **High Kd** → Improves damping but increases control effort.  

### 📌 **Noise Handling**  
- **Sensor noise** with standard deviation **σ = 0.05** is added to simulate inaccuracies.  
- **Low-pass filtering** ensures smooth sensor readings.  

### 📌 **Simulation Insights**  
- **Step Response Analysis**: The optimized PID controller achieves fast response with minimal overshoot.  
- **Noise-Added Simulation**: The PID controller effectively handles measurement noise.  
- **Degrees of Freedom Tuning**: Different PID variants (PID, PI-D, I-PD) were analyzed, and PID provided the best balance.  

---

## **System Analysis 🏛️**  

### 🔹 **Root Locus**  
- The system's stability is analyzed using **root locus plots**.  

### 🔹 **Bode Plot**  
- The **frequency response** is studied to assess system robustness.  

### 🔹 **State-Space Representation**  
- The system is also modeled using **state-space equations** for dynamic response analysis.  

---

## **Visualization 🎨**  

The project generates the following plots:  

📍 **Step Response**: System response with optimized PID parameters.  
📍 **Root Locus Plot**: Stability analysis of the open-loop system.  
📍 **Bode Plot**: Frequency response characteristics.  
📍 **Comparison of PID Variants**: Evaluating **PID, PI-D, and I-PD controllers**.  

---

## **Future Enhancements 🚀**  
🔹 Adaptive control strategies (LQR, Kalman Filters).  
🔹 AI-based PID tuning methods (Reinforcement Learning).  
🔹 Integration with real-world hardware (e.g., Arduino, Raspberry Pi).  

---

## **Contributing 🤝**  
Contributions are welcome! Feel free to:  
- **Fork the repo**  
- **Create a pull request**  
- **Report issues or suggest improvements**  

---

## **License 📜**  
This project is licensed under the **MIT License** – see the [LICENSE](LICENSE) file for details.  

---

This `README.md` ensures a **clear structure** and makes it easy for anyone exploring your repository. Let me know if you want any modifications! 🚀
