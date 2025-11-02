// =========================================================================
// Kinematics.h
// =========================================================================
#pragma once

// A simple struct to return the desired joint angles
struct JointAngles {
  float q1; // hip angle in radians
  float q2; // knee angle in radians
};

// --- Function Prototypes ---

// --- Core Controller Functions ---
JointAngles getJumpingAngles(float t);
JointAngles getStabilizingAngles();
JointAngles getFlightAngles();


// --- The Inverse Kinematics (IK) Solver ---
// This is the most important function you will write.
JointAngles solveInverseKinematics(float x_d, float y_d);