# 🦿 Jumping Humanoid Leg: Modeling, Dynamics, and Simulation

A comprehensive project focused on the **kinematic modeling, Lagrangian dynamics, and simulation** of a **two-link jumping humanoid leg**. This repository provides the framework for analyzing and controlling a high-performance robotic leg, paving the way for eventual hardware implementation of dynamic maneuvers like jumping.

---

## 🚀 Getting Started

These instructions will get you a copy of the project up and running on your local machine for simulation and development purposes.

### Prerequisites

The main simulation code is written in **MATLAB**.

* **MATLAB (R2025a or newer recommended)**
* **Symbolic Math Toolbox** (Required for running the dynamics derivation file)

### Installation and Execution

1.  **Clone the repository:**

 ```bash
 git clone https://github.com/jabichebli/Jumping-Humanoid-Leg.git
 cd Jumping-Humanoid-Leg
 ```

2.  **Generate Dynamic Files**
    **If the `auto` directory is empty**, execute this script to generate the symbolic functions (D, C, G matrices, Jacobians, etc.) required by the simulator.
    
 ```matlab
 % In MATLAB command window or script:
 simulate_jumping_leg
 ```

3.  **Run the Simulation:**
    Read through the documentation comments in `simulate_jumping_leg.m` script. Once happy, execute the main simulation file to run the ODE solver and see the animated results.
    
```matlab
% In MATLAB command window or script:
simulate_jumping_leg
```
    
---

## 📂 Repository Structure

The project is organized into logical directories for dynamics, simulation, and hardware code.
```
Jumping-Humanoid-Leg/
├── animate/                      # Scripts for visualizing the leg's movement
├── arduino_controller_code/      # C++ code for real-time control (e.g., servo control)
├── auto/                         # Automatically generated functions (D, C, G matrices, etc.)
├── dynamics/                     # Parameters and helper functions for the dynamic model
├── events/                       # Functions defining mode transitions (takeoff, touchdown)
├── media/                        # Storage for images, diagrams, and GIFs
├── other/                        # Miscellaneous files (development/ignore)
├── trajectory/                   # Files defining desired joint trajectories
├── generate_files.m              # Defines kinematics and dynamics symbolically and generates functions
├── simulate_jumping_leg.m        # Main file for running the jumping simulation (ODE solver)
└── README.md
```

### Key Files

| File | Purpose |
| :--- | :--- |
| `generate_files.m` | If the `auto` folder is empty, this must be run. This script sets up the kinematics and dynamics of the system symbolically and generates all required functions in the `auto` folder. |
| `simulate_jumping_leg.m` | This is the main file that defines the parameters of the leg, implements the ODE solvers, and manages the simulation states (stance/flight) and animation. |

---

## 📊 Simulation Results & Demos

### Controller Resilience & Perturbation Testing

A visualization of the leg's ability to withstand external perturbations (pushes).

<table width="100%">
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/6681fb43-3354-46d6-a241-8d7ace9f3639" alt="Vertical Push Animation GIF" width="100%">
      <br>
      <p>Vertical Push Animation</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/f0784f60-9b97-4812-87c6-edcd4fc86057" alt="Horizontal Push Animation GIF" width="100%">
      <br>
      <p>Horizontal Push Animation</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/78c10869-c005-46d5-a10f-c2a00fee0335" alt="Combination Push Animation GIF" width="100%">
      <br>
      <p>Combination Push Animation</p>
    </td>
  </tr>
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/4fcabac6-2c6c-459f-9846-2b9eea584a27" alt="Vertical Push Static Image" width="100%">
      <br>
      <p>Vertical Push Static Trajectory</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/40e6628c-246a-4c1e-9f72-2490f43cc2a9" alt="Horizontal Push Static Image" width="100%">
      <br>
      <p>Horizontal Push Static Trajectory</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/86ae061f-75b0-4db5-99aa-f204c9985538" alt="Combination Push Static Image" width="100%">
      <br>
      <p>Combination Push Static Trajectory</p>
    </td>
  </tr>
</table>

### Dynamic Jumping & Leaping

Visualizations showing the full dynamic jump sequence, controller performance, and horizontal leaps.

<table width="100%">
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/91e87833-7968-45ae-ae1f-abe7e8b06099" alt="Center of Mass Trajectory" width="100%">
      <br>
      <p>Center of Mass (CoM) Trajectory</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/3dcbd422-09e2-4f22-87dd-bf0192d7025e" alt="Hip States Position During Jump" width="100%">
      <br>
      <p>Hip State Variables During Jump</p>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/de2d2c77-0670-4aec-b44c-1f11c836d2ac" alt="Stable Balanced Stance" width="100%">
      <br>
      <p>Balanced Stance Demonstration</p>
    </td>
  </tr>
</table>
<table width="100%">
  <tr>
    <td align="center" width="50%">
      <img src="https://github.com/user-attachments/assets/e35b305f-7c8e-40ea-869e-99f3509b4769" alt="Forward Leap Animation" width="100%">
      <br>
      <p>Forward Leap Animation</p>
    </td>
    <td align="center" width="50%">
      <img src="https://github.com/user-attachments/assets/bfcf4916-9e60-49e7-bfe3-aa7743c1879f" alt="Backward Leap Animation" width="100%">
      <br>
      <p>Backward Leap Animation</p>
    </td>
  </tr>
</table>


---

## 💡 Future Work

<table width="100%">
  <tr>
    <td width="60%" valign="top">
      <p>A first-version physical prototype was developed. A jumping characteristic was achieved using a simplified joint angle controller that drives the leg to set values.</p><br>
      <p>If we had more time, and money, we would:</p>
      <ul>
        <li>Redesign the test-rig and mounting apparatus</li>
        <li>Implement an IMU and do IK and FK of the leg</li>
        <li>Use Brushless motors (to get force feedback for when we touch the ground)</li>
        <li>Transition the stabilizing controller and jumping controller implemented in MATLAB to C# (arduino) </li>
      </ul>
  </td>
    <td width="40%" align="center" valign="top">
      <img src="https://github.com/user-attachments/assets/443d9210-8300-4f32-b9cb-bd62fbb80690" alt="Jumping Real Leg GIF or Concept Image" width="100%">
      <br>
      <p><b>Physical Prototype/Concept Visual</b></p>
    </td>
  </tr>
</table>

---

## 📧 Contact

If you have questions, please open an issue on this repository or contact **[jabichebli](https://github.com/jabichebli)**.
