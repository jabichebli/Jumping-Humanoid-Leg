// =========================================================================
// Trajectory.h
// =========================================================================
#pragma once
#include "Config.h" // We need the trajectory parameters

// A simple struct to return the desired COM position
struct DesiredCOM {
  float x;
  float y;
};

// --- Function Prototypes ---

// Main function to port desired_jump_trajectory.m
DesiredCOM getDesiredCOM(float t);

// Helper function to port quintic_poly
float quintic_poly(float t, float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af);