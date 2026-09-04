# TEAM25-SIMULATE-HACKSIMUL8

## ML-Based Self-Tuning PID Control for a 6-DOF UAV Quadcopter

This project was developed for **HACKSIMUL8**, an 8-hour interdisciplinary hackathon conducted in collaboration with **MathWorks**.

The project develops a nonlinear 6-DOF quadcopter model in MATLAB/Simulink, designs an altitude PID controller, generates simulation data for PID optimization, and trains a machine-learning model to select PID gains according to the operating condition.

**Team:** TEAM 25 — SIMULATE

**Achievement:** 1st Place

---

## Project Overview

Quadcopter dynamics are nonlinear and controller performance can vary with different operating conditions. A fixed PID controller may therefore provide good performance for one condition but degrade under other initial and commanded altitude conditions.

This project implements an ML-based self-tuning approach in which the operating condition is used to predict suitable PID gains.

The overall workflow is:

```text
6-DOF Quadcopter Dynamics
          ↓
Altitude PID Controller
          ↓
Simulation Experiments
          ↓
Scenario-Specific PID Optimization
          ↓
Training Dataset
          ↓
ML Regression Model
          ↓
Predicted PID Gains
          ↓
Self-Tuned Altitude Control
```
---

## Tasks
### Task 1 — 6-DOF Quadcopter Dynamic Model

A nonlinear 12-state model of the quadcopter was implemented in MATLAB/Simulink.

The state vector is:
`[x, y, z, x_dot, y_dot, z_dot,
 phi, theta, psi, phi_dot, theta_dot, psi_dot]`

The model includes:

- Translational dynamics
- Rotational dynamics
- Euler-angle representation
- Inertia properties
- Rotor-generated thrust
- Rotor-generated roll, pitch, and yaw torques
- Nonlinear coupling between translational and rotational motion

The quadcopter parameters are based on the [reference paper](https://doi.org/10.31763/ijrcs.v4i4.1594) used for the project.

The resulting nonlinear plant is implemented inside - `model/Quadcopter_Dynamics_ver2.slx`

The Simulink model contains the `quad_dynamics` MATLAB Function block and the rotor `hover_mixer`.

---

## Task 2 — Altitude PID Control


- The altitude controller regulates the vertical position z.

- The altitude error is `e = zd - z`; where zd = desired altitude and z0 = actual altitude

- The PID controller generates the commanded thrust `F_cmd`. The commanded thrust is passed to the hover mixer, which calculates equal rotor speeds for the nominal altitude-control case.

### Baseline PID Gains
**Kp = 56.8136\
Ki = 7.3365\
Kd = 31.7435**

- These gains were obtained from the reference paper using the Tyreus-Luyben tuning method.

- The baseline controller provides the reference for the ML-based self-tuning comparison.

---

## Task 3 — Simulation Data Generation and ML Training 

### PID Optimization Dataset

- Simulation experiments were generated over different combinations of:

    - Initial altitude z0
    - Desired altitude zd

- The final dataset contains 22 operating scenarios.

- For each scenario, the PID gains are locally optimized using MATLAB's fminsearch optimizer.

- The optimization objective combines:

    - ITAE (Integral of Time-weighted Absolute Error)
    - Overshoot penalty

- The resulting dataset contains:
    - z0 - initial altitude
    - zd - target altitude
    - Kp, Ki, Kd
    - overshoot peak time
    - rise_time
    - settling_time
    - ITAE
    - cost

1. The generated dataset is: `data/altitude_pid_dataset.csv`

2. The generation script is: `src/task3_generate_data.m`

### ML Model

The ML model learns the relationship between the operating condition and the optimized PID gains:

- Inputs:
[z0, zd]

- Outputs:
[Kp, Ki, Kd]

A separate regression model is trained for each PID gain.

When available, MATLAB fitrnet is used with:

- Two hidden layers: [10 6]
- ReLU activations
- Standardized inputs
- Regularization
- 80/20 hold-out testing
- Random seed: 42

1. The training script is: `src/task3_train_ml_model.m`

2. The trained model is saved as: `data/pid_tuner_net.mat`

---

## Task 4 — ML-Based Self-Tuning PID

The trained ML model is used to predict PID gains for new operating conditions.

### The Task 4 implementation:

- Defines the initial altitude z0 and desired altitude zd.
- Loads the trained ML models.
- Predicts Kp, Ki, and Kd.
- Applies the predicted gains to the Simulink PID controller.
- Runs the nonlinear quadcopter simulation.
- Measures altitude-control performance.
- Compares the ML-tuned controller with the fixed baseline controller.

The current implementation performs gain prediction once at the beginning of each test case. It is therefore a mission-start self-tuning approach rather than continuous in-flight adaptation.

**The implementation is: `src/task4_selftuning_pid.m`**

### Task 4 Test Cases

Four operating conditions were evaluated.

| Case | z0 (m) | zd (m) | Test Type |
|---|---:|---:|---|
| 1 | 0.10 | 1.75 | Interpolated / off-grid |
| 2 | 0.75 | 3.50 | Interpolated / off-grid |
| 3 | 0.00 | 2.00 | Validated baseline sanity case |
| 4 | 0.00 | 4.75 | Extrapolation beyond training range |

**Performance Comparison**

Across all four test cases, the ML-tuned controller improved:

- Overshoot: 4/4 cases
- Rise time: 4/4 cases
- Settling time: 4/4 cases
- ITAE: 4/4 cases


For the complete Task 4 results and simulation output, see: `results/TASK4_results.md`

---
## Model Assumptions

The implementation follows the assumptions specified for the quadcopter model:
- Homogeneous and symmetric quadcopter
- Cross-shaped configuration
- Center of gravity at the body-frame origin
- Rigid transmission
- Fixed model parameters
- No external translational acceleration inputs (Ax = Ay = Az = 0) because numerical values were not specified in the reference model
- Practical ground constraint is included in the simulation
- PID thrust command is limited to the implemented controller saturation range

---

## Repository Structure
```text
TEAM25-SIMULATE-HACKSIMUL8/
│
├── README.md
├── .gitignore
│
├── model/
│   └── Quadcopter_Dynamics_ver2.slx
│
├── src/
│   ├── task3_generate_data.m
│   ├── task3_train_ml_model.m
│   └── task4_selftuning_pid.m
│
├── data/
│   ├── altitude_pid_dataset.csv
│   └── pid_tuner_net.mat
│
└── results/
    ├── TASK4_results.md
    ├── task4_comparison_results.csv
    └── figures/
        ├── case1_z0_0.10_zd_1.75.png
        ├── case2_z0_0.75_zd_3.50.png
        ├── case3_z0_0.00_zd_2.00.png
        └── case4_z0_0.00_zd_4.75.png
```
--- 

## Software Requirements
1. MATLAB
2. Simulink
3. Statistics and Machine Learning Toolbox recommended for fitrnet
4. Optimization Toolbox for fminsearch

The model and scripts were developed for MATLAB/Simulink-based simulation.

---
## Files
1. Simulink model: model/Quadcopter_Dynamics_ver2.slx
2. Dataset: data/altitude_pid_dataset.csv
3. Trained ML model: data/pid_tuner_net.mat
4. Task 3 data generation: src/task3_generate_data.m
5. Task 3 ML training: src/task3_train_ml_model.m
6. Task 4 implementation: src/task4_selftuning_pid.m
7. Task 4 numerical results: results/task4_comparison_results.csv
8. Detailed Task 4 results: results/TASK4_results.md