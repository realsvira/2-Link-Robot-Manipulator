# Two-Link Robot Manipulator - Systems Thinking Project

A comprehensive implementation and analysis of a two-link planar robotic manipulator using Lagrangian mechanics and PID control in MATLAB/Simulink.

## 📋 Overview

This project implements a dynamically rich two-link robotic arm modeled as a planar double pendulum system. The manipulator features two degrees of freedom with revolute joints, making it an ideal platform for studying control algorithms and dynamic modeling techniques in robotics.

**Team:** SYSTUMMM THINKERS
- Madhur Kankane (2024102061)
- Het Selarka (2024102031)
- Shrenil Patel (2024102066)
- Siddhant Gudwani (2024102042)
- Saumya Vira (2024102044)
- Anish Toshniwal (2024102009)

## 🎯 Key Features

- **Complete Dynamic Modeling**: Lagrangian formulation with coupled nonlinear differential equations
- **State-Space Representation**: Full derivation of inertia (M), Coriolis (C), and gravity (G) matrices
- **Multiple Control Strategies**: Implementation of P, PD, PI, and PID controllers
- **Three Tuning Philosophies**: 
  - Balanced (optimal trade-off)
  - Aggressive (maximum speed)
  - Conservative (maximum stability)
- **Comprehensive Analysis**: Performance metrics including settling time, overshoot, and steady-state error
- **Simulink Implementation**: Visual block diagram with modular subsystems

## 📐 System Parameters
```matlab
Link 1: mass (m1) = 5 kg, length (l1) = 0.25 m
Link 2: mass (m2) = 3 kg, length (l2) = 0.15 m
Gravitational acceleration: g = 9.81 m/s²
Initial conditions: [q1(0), q2(0)] = [0.1, 0.1] rad
Target position: [q1, q2] = [0, 0] rad
```

## 🚀 Getting Started

### Prerequisites

- MATLAB R2025a or later
- Simulink
- Control System Toolbox

### Running the Simulation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/two-link-robot-manipulator.git
cd two-link-robot-manipulator
```

2. Open MATLAB and navigate to the project directory

3. Run the main simulation:
```matlab
run('PID_code.m')
```

4. Open the Simulink model:
```matlab
open('SystummThinkers_Simulink.slx')
```

## 📊 Results & Analysis

### Controller Performance Comparison

| Controller | Settling Time (q1) | Overshoot (q1) | Steady-State Error |
|------------|-------------------|----------------|-------------------|
| PI | Unstable | 382.94% | Failed |
| PD | Moderate | 46.37% | 0.0435 |
| PID (Balanced) | 65.234 ms | 0.00% | 0.0000 |

### Key Findings

- **PID Controller** demonstrates optimal performance with zero steady-state error and minimal overshoot
- **PD Controller** provides good stability but cannot eliminate steady-state error
- **PI Controller** is unsuitable for this dynamic system due to lack of damping
- **Balanced tuning** (Kp=120, Ki=180, Kd=150) achieves best overall performance

## 📁 Project Structure
```
├── Report.pdf                      # Complete project documentation
├── Dynamics.m                      # Dynamic equations function
├── PID_code.m                      # Main simulation script
├── SystummThinkers_Simulink.slx   # Simulink model
└── README.md                       # This file
```

## 🔬 Mathematical Framework

### Lagrangian Formulation
```
L = KE - PE
KE = ½M₁L₁²θ̇₁² + ½M₂[L₁²θ̇₁² + L₂²(θ̇₁ + θ̇₂)² + 2L₁L₂θ̇₁(θ̇₁ + θ̇₂)cos(θ₂)]
PE = (M₁ + M₂)gL₁sin(θ₁) + M₂gL₂sin(θ₁ + θ₂)
```

### Equations of Motion
```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
```

### PID Control Law
```
τ = M(q)[Kp·e + Ki·∫e·dt + Kd·ė]
where e = qdesired - qactual
```

## 📈 Visualization

The simulation generates plots for:
- Joint angle trajectories (q1 vs q2)
- Joint velocities (q̇1 vs q̇2)
- Control torques (τ1 vs τ2)
- Error signals over time
- Phase portraits

## 🔧 Tuning Guidelines

### Balanced Tuning (Recommended)
```matlab
Kp = [120, 120]
Ki = [180, 180]
Kd = [150, 150]
```

### Aggressive Tuning (Fast Response)
```matlab
Kp = [200, 200]
Ki = [400, 400]
Kd = [20, 20]
```

### Conservative Tuning (High Precision)
```matlab
Kp = [40, 40]
Ki = [20, 20]
Kd = [60, 60]
```

## 📚 References

- Lagrangian Mechanics in Robotics
- PID Control Theory
- Nonlinear Dynamics and Chaos
- Robot Modeling and Control (Spong, Hutchinson, Vidyasagar)

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is part of an academic course at IIIT Hyderabad. Please contact the authors for usage rights.

## 📧 Contact

For questions or collaboration:
- Het Selarka: het.selarka@students.iiit.ac.in

## 🙏 Acknowledgments

- International Institute of Information Technology, Hyderabad
- Systems Thinking Course Instructors
- MATLAB & Simulink Documentation

---

**Note**: This project demonstrates fundamental concepts in robotics, control theory, and systems modeling. The PID controller achieves excellent performance for this two-link system, though more advanced controllers (adaptive, learning-based) could be explored for complex scenarios.
