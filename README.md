# 🛰️ SDP-Based Satellite Attitude Control

Constrained spacecraft attitude control using **Semidefinite Programming (SDP)** with **quaternion** dynamics. The controller tracks a desired orientation while respecting geometric keep-in/keep-out cones and actuator limits. Simulations are in MATLAB/Simulink; optimization is solved via MOSEK through YALMIP or CVX.

## 🚀 Overview
- **Method:** SDP (MOSEK via **YALMIP** or **CVX**)
- **States:** Unit quaternions \(q\), body rates \(\omega\)
- **Constraints:** Keep-in/keep-out cones, bounds on \(\|\omega\|\) and torque \(u\), terminal state
- **Env:** MATLAB & Simulink

## 📂 Features
- Rigid-body (Euler) rotational dynamics + quaternion kinematics
- Exact cone constraints via SDP
- Plots for boresight on sphere, constraint margins, attitude/rate histories
- Baseline PD vs. SDP-guided tracking comparison

## 🧮 Dependencies
- MATLAB R2021b+ and Simulink  
- **Modeling:** [YALMIP](https://yalmip.github.io/download/) or [CVX](https://cvxr.com/cvx/download/)  
- **Solver:** [MOSEK](https://www.mosek.com/downloads/) (installed, licensed, on MATLAB path)  
  - Using CVX? Set `cvx_solver mosek`.

## 🛠️ How to run
1. **Open the repo in MATLAB** and set the current folder to the repo root.  
2. **Add paths:** *Home → Set Path → Add with Subfolders…* → select repo root → **Save**.  
3. **Configure a scenario:** edit `Code/initialize_satellite_simulation.m` (desired quaternion, torque/rate limits, cone angles).  
4. **Solve (SDP):** run `initialize_satellite_simulation` (calls `solveSDPControl.m`). Results are written to `Results/<run_name>`.  
5. **Plot:** run `plot_SDPsim_results.m` (or `plot_SDP_sim_results.mlx`) to save figures in `Results/<run_name>`.  
6. **(Optional) Simulink tracking:** open `CubeSat_SDP_PD_Attitude.slx`, select **SDP reference** (or baseline PD), and **Run**.  
   - *What Simulink does:* a quaternion-PD tracker compares measured attitude to the SDP reference, applies torque limits, and integrates Euler/quaternion dynamics; logs (`q`, `ω`, `u`, errors, margins) go to `Results/<run_name>`.  
7. **Reproduce example bundles:** folders under `Results/` (e.g., `1ki_*`, `1ko_*`, `2ki_*`, `2ko_*`, `no_constraint_plots`) use the same steps with different cone settings.

## 📜 License
MIT License

---

Made with ☕ and 🚀 by **Shavy Kashyap**
