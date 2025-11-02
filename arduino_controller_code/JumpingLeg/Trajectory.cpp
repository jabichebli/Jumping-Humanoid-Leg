// =========================================================================
// Trajectory.cpp
// =========================================================================
#include "Trajectory.h"
#include <Arduino.h> // For pow(), sin(), cos(), PI

// This is the C++ port of the `quintic_poly` sub-function
// It is used for the X-axis (lean) trajectory.
float quintic_poly(float t, float t0, float tf, float p0, float pf, float v0, float vf, float a0, float af) {
  float T = tf - t0;
  if (T <= 0) {
    return pf;
  }
  
  // This is a (t-t0) polynomial, but written in a normalized way.
  // (tau * T) is just (t-t0). This is correct.
  float tau = (t - t0) / T;

  // Coefficients from the MATLAB file
  float c0 = p0;
  float c1 = v0;
  float c2 = 0.5 * a0;
  float c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*pow(T,2)) / (2*pow(T,3));
  float c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*pow(T,2)) / (2*pow(T,4));
  float c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*pow(T,2)) / (2*pow(T,5));

  // Note: pow(tau*T, 2) is just pow(t-t0, 2). This is correct.
  float p = c0 + c1*tau*T + c2*pow(tau*T,2) + c3*pow(tau*T,3) + c4*pow(tau*T,4) + c5*pow(tau*T,5);
  return p;
}


// =========================================================================
// Main trajectory function (Point 4a)
// This is a direct port of desired_jump_trajectory.m
// =========================================================================
DesiredCOM getDesiredCOM(float t) {
  DesiredCOM desired;
  float x_d, y_d;

  // --- 1. Setup the Problem (from MATLAB) ---
  
  // Get trajectory parameters from Config.h
  // T_HOLD, T_JUMP, T_DIP_RATIO, Y_STAND, Y_SQUAT, Y_TAKEOFF,
  // X_LEAN (from x_squat_lean), X_TAKEOFF
  
  // This value was in simulate_jumping_leg.m but not in Config.h
  // We define it here.
  const float VF_TAKEOFF = 0.6; // from params.vf_takeoff

  // Compute phase durations
  float T_dip  = T_JUMP * T_DIP_RATIO;
  float T_push = T_JUMP * (1.0 - T_DIP_RATIO);
  
  // Assign absolute times for phase transitions
  float t_end_hold = T_HOLD;
  float t_end_dip  = T_HOLD + T_dip;
  float t_end_push = t_end_dip + T_push; // This is (T_HOLD + T_JUMP)

  // --- 2. Calculate Trajectory (X and Y) based on phase ---

  if (t < t_end_hold) {
    // ---------------------- Phase 1: Initial Hold --------------------
    y_d = Y_STAND;
    x_d = 0.0;
  
  } else if (t < t_end_dip) {
    // ---------------------- Phase 2a: Downward Dip / Lean -------------------
    
    // Y-Trajectory (sin/cos dip from MATLAB)
    float tau_dip = t - t_end_hold;
    float A_dip = (Y_STAND - Y_SQUAT) * (2.0 * PI) / (T_dip * T_dip);
    float sin_term = sin(2.0 * PI * tau_dip / T_dip);
    y_d = Y_STAND + (A_dip * T_dip * T_dip / (4.0 * PI * PI)) * sin_term 
                  - (A_dip * T_dip / (2.0 * PI)) * tau_dip;

    // X-Trajectory (lean to squat from MATLAB)
    x_d = quintic_poly(t, t_end_hold, t_end_dip, 
                       0.0, X_LEAN, 0, 0, 0, 0);
  
  } else if (t < t_end_push) {
    // ---------------------- Phase 2b: Push-Off -----------------------

    // Y-Trajectory (Custom quintic from MATLAB)
    float tau_push = t - t_end_dip;
    float s = tau_push / T_push;
    float P_end = Y_TAKEOFF - Y_SQUAT;    // Total displacement
    float V_end = VF_TAKEOFF * T_push;    // Scaled target velocity
    
    // Quintic polynomial coefficients for normalized s ∈ [0,1]
    float c3 = 10.0*P_end - 4.0*V_end;
    float c4 = -15.0*P_end + 7.0*V_end;
    float c5 = 6.0*P_end - 3.0*V_end;
    // Normalized powers of s
    float s2 = s*s, s3 = s*s2, s4 = s*s3, s5 = s*s4;
    // Compute normalized position
    float p_norm = c3*s3 + c4*s4 + c5*s5;
    // Scale back to physical units
    y_d = Y_SQUAT + p_norm;

    // X-Trajectory (lean to takeoff from MATLAB)
    x_d = quintic_poly(t, t_end_dip, t_end_push, 
                       X_LEAN, X_TAKEOFF, 0, 0, 0, 0);
  
  } else {
    // ---------------------- Phase 3: Trajectory Finished -----------------
    // The main loop will switch to FLIGHT state.
    // We just command the final takeoff pose.
    y_d = Y_TAKEOFF;
    x_d = X_TAKEOFF;
  }

  // --- 3. Output Results ---
  desired.x = x_d;
  desired.y = y_d;
  
  return desired;
}
