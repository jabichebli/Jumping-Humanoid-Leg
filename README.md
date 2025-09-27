# Jumping-Humanoid-Leg

Kinematic modelling, dynamics, and simulation of of a two link leg.

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
