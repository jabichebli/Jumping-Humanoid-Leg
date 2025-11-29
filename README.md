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
    git clone [https://github.com/jabichebli/Jumping-Humanoid-Leg.git](https://github.com/jabichebli/Jumping-Humanoid-Leg.git)
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

*(Please replace these placeholders with actual images or videos demonstrating your simulation results!)*

### Stance Phase and Jump Sequence

A visualization of the full jump sequence, showing the compression in the stance phase, the push-off, and the resulting flight phase.

[GIF of the simulated jumping leg motion]

### Balancing and Stability

An image showing the leg maintaining a stable, balanced stance position using a feedback controller.



---

## 💡 Future Work

The following features are planned for future development to enhance the model's accuracy and robustness:

* **Virtual Constraints:** Implement virtual constraints to ensure the Center of Mass (**CoM**) remains above the stance foot position during the entire ground contact phase.
* **Exit Velocity Calculation:** Calculate the necessary exit velocity and use it as an event trigger to smoothly transition from the stance phase to the flight phase in the ODE solver.
* **Ground Constraints:** Implement a more robust physical constraint model to prevent the stance foot from penetrating the ground.
* **Joint Friction:** Integrate joint friction models to better match real-world physical behavior.

---

## 📄 License

This project is licensed under the MIT License - see the `LICENSE` file for details.

## 📧 Contact

If you have questions, please open an issue on this repository or contact the maintainer: [jabichebli]
