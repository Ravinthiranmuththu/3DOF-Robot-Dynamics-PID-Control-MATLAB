# Dynamic Modelling and PID Control of a 3-DOF Robotic Manipulator

**MATLAB · Simulink · Robot kinematics · Lagrangian dynamics · PID control**

An academic robotics and control-systems assignment developed for **EE7215/EC7215 Robotics and Automation**, University of Ruhuna. The project explores the modelling and simulation of a **three-revolute-joint (RRR) planar manipulator** and feedback control for position and velocity trajectories.

> **Scope:** This repository presents the original submitted MATLAB script and Simulink models, organized for review. It has not been independently rerun or validated in MATLAB in this packaging step.

## Demonstration

[Watch the project showcase (MP4)](Robotics_Showcase.mp4)

The video is an edited overview. It should not be treated as quantitative proof of controller performance; inspect the model scope outputs and reproduce the simulations to assess tracking and disturbance rejection.

## Repository contents

| Path | Description |
| --- | --- |
| `PartA.m` | Symbolic kinematics, Jacobian, energies, and Euler–Lagrange derivation |
| `PartB.slx` | PID-based **joint-space position** tracking |
| `PartC.slx` | PID-based **task-space position** tracking |
| `PartD.slx` | PID-based **joint-space velocity** tracking |
| `Robotics_Showcase.mp4` | Short edited simulation showcase |

## Technical overview

### Part A — Kinematics and dynamics

The original `PartA.m` defines symbolic joint coordinates, velocities, and accelerations; constructs homogeneous transforms; computes an end-effector position Jacobian; and formulates kinetic energy, potential energy, and Euler–Lagrange equations.

The model uses three links of **0.5 m** and **1 kg** each, with symbolic inertia. The script takes gravity along the negative z-axis, yielding zero gravitational potential for the planar configuration used in the submitted report.

### Part B — Joint-space position control

A Simulink model for tracking desired joint-angle trajectories with PID-based feedback.

### Part C — Task-space position control

A Simulink model for tracking an end-effector position trajectory using PID-based feedback.

### Part D — Joint-space velocity control

A Simulink model for tracking desired joint-velocity trajectories using PID-based feedback.

The assignment also investigates the controllers' response to an applied joint disturbance. No quantitative tracking-error metrics or robustness claims are made here without a reproducible rerun.

## Requirements

- MATLAB
- Simulink
- Symbolic Math Toolbox (for the symbolic computations in `PartA.m`)
- Any additional toolboxes or dependencies used internally by the Simulink models, as applicable

**MATLAB release/version was not documented in the supplied archive.**

## How to explore

1. Clone or download the repository.
2. Open MATLAB and set the working directory to the repository root.
3. Inspect `MATLAB/PartA.m` and the models in `Simulink/`.
4. Open each `.slx` file and inspect its blocks, inputs, workspace variables, and solver configuration before running.
5. Provide any missing helper functions or trajectory scripts (see below), then run the simulations in a compatible MATLAB/Simulink environment.

### Known limitations and missing dependencies

The submitted archive contains `PartA.m` and the three `.slx` files, but **does not contain** the separately named desired-trajectory `.m` scripts referenced in the assignment brief:

- `B_Desired_Jointspace_Position_Trajectory.m`
- `C_Desired_Taskspace_Position_Trajectory.m`
- `D_Desired_Jointspace_Velocity_Trajectory.m`

`PartA.m` also calls `plot_RRR(...)`, whose definition was not present as a standalone file in the archive. These items **have not been invented or silently replaced**. The Simulink models may contain embedded trajectory definitions or require additional workspace initialization; inspect each model to determine its actual dependencies.

**Important:** The original `PartA.m` is preserved unchanged. Its analytical implementation should be independently reviewed before relying on it as a verified robot dynamics model.

## Academic context

Coursework project, **Robotics and Automation (EE7215/EC7215)**, Department of Electrical and Information Engineering, University of Ruhuna.

## Attribution and reuse

This is an educational portfolio project. No open-source license is granted in this repository by default. Please check applicable course policies before redistributing the assignment or reusing the code.
