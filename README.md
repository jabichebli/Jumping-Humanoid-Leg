# 🤖 Jumping Humanoid Leg: Modeling, Dynamics, and Simulation

A comprehensive project focused on the **kinematic modeling, Lagrangian dynamics, and simulation** of a **two-link jumping humanoid leg**. This repository provides the framework for analyzing and controlling a high-performance robotic leg, paving the way for eventual hardware implementation of dynamic maneuvers like jumping.

---

## ✨ Features

* **Lagrangian Dynamics Derivation:** Automated symbolic calculation of the mass matrix ($\mathbf{D}$), Coriolis/Centrifugal terms ($\mathbf{C}$), and Gravity terms ($\mathbf{G}$) for the two-link system.
* **Dynamic Simulation:** Utilizes an ODE solver (MATLAB's `ode45`) to simulate the leg's movement during both the stance (ground contact) and flight phases.
* **Trajectory Planning:** Code for generating and testing desired joint angle trajectories.
* **Animation:** Tools to visualize the simulated leg motion for clear analysis.
* **Hardware Control Foundation:** Includes initial C/C++ code for potential real-world deployment using an Arduino or similar microcontroller.

---

## 📐 Kinematics & Dynamics Model

The project is built upon a standard two-link planar leg model, often simplified as an inverted pendulum during the stance phase.

The system dynamics are derived using the **Lagrangian method**, resulting in the general form:
$$
\mathbf{D}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \boldsymbol{\tau} + \mathbf{J}^T \mathbf{F}_{ext}
$$
Where:
* $\mathbf{q}$ is the vector of generalized coordinates (joint angles).
* $\mathbf{D}$ is the Mass Matrix.
* $\mathbf{C}$ contains the Coriolis and Centrifugal terms.
* $\mathbf{G}$ is the Gravity vector.
* $\boldsymbol{\tau}$ is the vector of joint torques.
* $\mathbf{J}$ is the Jacobian matrix of the stance foot.
* $\mathbf{F}_{ext}$ are external forces (e.g., ground reaction force).

The Free Body Diagram below illustrates the two-link model used for the derivation:



---

## 🚀 Getting Started

These instructions will get you a copy of the project up and running on your local machine for simulation and development purposes.

### Prerequisites

The main simulation code is written in **MATLAB**.

* **MATLAB (2018b or newer recommended)**
* **Symbolic Math Toolbox** (Required for running the dynamics derivation file)

The hardware controller code requires:
* **Arduino IDE** (or similar C/C++ environment)

### Installation and Execution

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/jabichebli/Jumping-Humanoid-Leg.git
    cd Jumping-Humanoid-Leg
    ```

2.  **Generate Dynamic Functions:**
    The core dynamic equations are generated symbolically to optimize simulation speed. This step must be run first.
    ```matlab
    % In MATLAB command window or script:
    LagrangianDynamics
    ```
    This script calculates $\mathbf{D}, \mathbf{C}, \mathbf{G}$, Jacobians, etc., and saves them as `auto_*.m` functions in the `auto` folder.

3.  **Run the Simulation:**
    Execute the main simulation file to run the ODE solver and see the animated results.
    ```matlab
    % In MATLAB command window or script:
    simulate_jumping_leg
    ```
    *(For testing stance control, you can also run `simulate_leg_balance_stance`)*

---

## 📂 Repository Structure

The project is organized into logical directories for dynamics, simulation, and hardware code.
```
Jumping-Humanoid-Leg/
├── animate/                      # Scripts for visualizing the leg's movement
├── arduino_controller_code/      # C/C++ code for real-time control (e.g., servo control)
├── auto/                         # Automatically generated functions (D, C, G matrices, etc.)
├── dynamics/                     # Parameters and helper functions for the dynamic model
├── media/                        # Storage for images, diagrams, and GIFs
├── trajectory/                   # Files defining desired joint trajectories
├── LagrangianDynamics.m          # The core script to symbolically derive and generate dynamics
├── simulate_jumping_leg.m        # Main file for running the jumping simulation (ODE solver)
└── README.md                     # This file
```

### Key Files

| File | Purpose |
| :--- | :--- |
| `LagrangianDynamics.m` | Calculates the Lagrangian dynamics ($\mathbf{D}, \mathbf{C}, \mathbf{G}$) and saves the resulting functions for simulation. |
| `simulate_jumping_leg.m` | Implements the ODE solver for the leg dynamics, integrates the equations of motion, and manages the simulation states (stance/flight). |
| `simulate_leg_balance_stance.m`| A dedicated script for simulating the balancing or stable stance phase. |
| `generate_files.m` | Utility script called by `LagrangianDynamics.m` to clean up and organize generated functions. |

---

## 🖼️ Results and Demos

 ![animation_Vertical_Pushes](https://github.com/user-attachments/assets/6681fb43-3354-46d6-a241-8d7ace9f3639)
<img width="602" height="767" alt="Vertical_Push" src="https://github.com/user-attachments/assets/4fcabac6-2c6c-459f-9846-2b9eea584a27" />
![animation_Horizontal_Pushes](https://github.com/user-attachments/assets/f0784f60-9b97-4812-87c6-edcd4fc86057)
<img width="602" height="767" alt="Horizontal_Push" src="https://github.com/user-attachments/assets/40e6628c-246a-4c1e-9f72-2490f43cc2a9" />


### Stance Phase and Jump Sequence

A visualization of the full jump sequence, showing the compression in the stance phase, the push-off, and the resulting flight phase.



### Balancing and Stability

An image showing the leg maintaining a stable, balanced stance position using a feedback controller.
![jumping_leg_1](https://github.com/user-attachments/assets/de2d2c77-0670-4aec-b44c-1f11c836d2ac)
<img width="602" height="868" alt="Hip_States_Position_During_Jump" src="https://github.com/user-attachments/assets/3dcbd422-09e2-4f22-87dd-bf0192d7025e" />
<img width="602" height="749" alt="COM_Trajectory" src="https://github.com/user-attachments/assets/91e87833-7968-45ae-ae1f-abe7e8b06099" />

![leg_leaping_animation_backwards_leap](https://github.com/user-attachments/assets/bfcf4916-9e60-49e7-bfe3-aa7743c1879f)
![leg_leaping_animation_forward_leap](https://github.com/user-attachments/assets/e35b305f-7c8e-40ea-869e-99f3509b4769)


---

## 💡 Future Work

A first-version physical prototype was developed. A jumping characteristic was achieved using a simplified joint angle controller that drives the leg to set values.

If we had more time, and money, we would:
- Redesign the test-rig and mounting apparatus
- Implement an IMU and do IK and FK of the leg
- Use Brushless motors (to get force feedback for when we touch the ground)
- Transition the stabilizing controller and jumping controller implemented in MATLAB to C# (arduino) 

---

## 📧 Contact

If you have questions, please open an issue on this repository or contact the maintainer: [jabichebli]
