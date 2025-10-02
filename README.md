# Jumping-Humanoid-Leg

Kinematic modelling, dynamics, and simulation of of a two link leg.

## Kinematics:
<img width="1603" height="1024" alt="Free_Body_Diagram" src="https://github.com/user-attachments/assets/37b2fd00-dd50-42ff-9bea-ba8405575192" />

## Todos:

- [ ] Add in virtual constraints for keeping CoM above stance foot position
- [ ] Add constraints for other parts to not go below the ground?
- [ ] Calculate exit velocity and use as event to go from stance to flight
- [ ] Add friction to joints
- [x] Implement kinematics for leg (positions)
- [x] Implement dynamics for leg (lagrangian)
- [x] Implement basic simulation/animation
- [x] Add physical constraint for stance foot to not go into the ground

## To run:
1. run dynamics_leg.m (this generates functions that are used in simulate_leg)
3. run simulate_leg.m

## Purpose of Each File

#### dynamics_leg.m

- Calculates lagrangian dynamics of system (D, C, G, B)
- Calculates jacobian of stance foot, and derivative of jacobian of stance foot
- Generates auto_{variable}.m files to be used in other functions for speed

#### simulate_leg.m

- Implements ODE solver for leg dynamics
- Simulates leg movement with animation

#### position_testing.m

- Used as sanity check for position calculations
