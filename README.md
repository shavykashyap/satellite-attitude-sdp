# satellite-attitude-sdp

**Semidefinite Programming (SDP)-Based Satellite Attitude Control**
This project implements a constrained quaternion-based attitude control system for a satellite using semidefinite programming(SDP). The goal is to achieve precise target tracking while respecting sun-avoidance constraints through convex optimization and geometric control techniques. Simulations are developed in MATLAB/Simulink.


# 🛰️ SDP-Based Satellite Attitude Control

This repository implements a **semidefinite programming (SDP)**-based controller for satellite attitude regulation using **quaternion dynamics**. The control system is designed to track a desired orientation while respecting geometric constraints — such as maintaining angular separation from the Sun (e.g., for sun-avoidance maneuvers in sensitive instruments).

## 🚀 Project Overview

- **Control Method:** Semidefinite Programming (via YALMIP and MOSEK)
- **State Representation:** Quaternions for 3D attitude
- **Constraints:** Minimum angular separation between the satellite boresight vector and the Sun vector
- **Simulation Environment:** MATLAB & Simulink

## 📂 Features

- Quaternion-based rigid body dynamics simulation
- Sun-exclusion zone enforced through SDP constraints
- Visual animation of satellite orientation, boresight cone, and trajectory
- Comparison framework with traditional feedback control
- Early termination based on quaternion error threshold


## 🧮 Dependencies

- MATLAB R2021b or later
- Simulink
- [YALMIP](https://yalmip.github.io/download/)
- [MOSEK](https://www.mosek.com/downloads/)

<!-- ## 🎥 Example Output

> A sample animation showing the satellite cone tracking and constraint enforcement will be added here. -->

<!-- ## 🛠️ TODO

- Add support for different Sun vector trajectories
- Compare with LQR and PD controllers
- Improve real-time visualization for feedback+SDP hybrid -->

## 📜 License

MIT License 

---

Made with ☕ and 🚀 by SHAVY KASHYAP
