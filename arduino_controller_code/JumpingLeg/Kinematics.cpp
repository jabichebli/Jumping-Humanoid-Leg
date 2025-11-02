// =========================================================================
// Kinematics.cpp
// =========================================================================
#include "Kinematics.h"
#include "Trajectory.h" // Needs this to get desired COM
#include "Config.h"     // Needs this for setpoints and link lengths
#include <Arduino.h>    // For cos(), acos(), sqrt(), atan2()

// Controller for JUMPING state (Point 4)
JointAngles getJumpingAngles(float t) {
  // 1. Get desired COM position from trajectory
  DesiredCOM p_d = getDesiredCOM(t);
  
  // 2. Solve IK to get required joint angles
  //    (We assume p_COM_d is our desired HIP position)
  return solveInverseKinematics(p_d.x, p_d.y);
}

// Controller for STABILIZING and LANDED states (Point 6)
JointAngles getStabilizingAngles() {
  // 1. Define a stable COM position (e.g., standing straight)
  float x_stable = X_LEAN;
  float y_stable = Y_STAND;

  // 2. Solve IK for this static point
  return solveInverseKinematics(x_stable, y_stable);
}

// Controller for FLIGHT state
JointAngles getFlightAngles() {
  JointAngles q;
  q.q1 = FLIGHT_HIP_ANGLE_RAD;
  q.q2 = FLIGHT_KNEE_ANGLE_RAD;
  return q;
}


// =========================================================================
// INVERSE KINEMATICS (IK) SOLVER
// =========================================================================
// This is the completed analytical 2-link IK solver.
// It assumes:
// - Foot is at [0, 0]
// - L1 (thigh) and L2 (shank) are the link lengths.
// - Input: [x_d, y_d] is the desired HIP position.
// - Output: [q1, q2] are the servo angles.
//   - q1: Absolute hip angle from vertical (y-axis).
//   - q2: Relative knee angle (relative to the thigh).

JointAngles solveInverseKinematics(float x_d, float y_d) {
  JointAngles q;

  // Link lengths from config
  // L1 = Thigh, L2 = Shank
  
  // 1. Calculate the distance from foot (origin) to hip
  float L_hip_sq = x_d * x_d + y_d * y_d;
  float L_hip = sqrt(L_hip_sq);

  // 2. Calculate the relative knee angle (q2) using Law of Cosines
  // L_hip^2 = L1^2 + L2^2 - 2*L1*L2*cos(alpha_knee)
  // where alpha_knee is the *interior* angle.
  // The servo angle (q2) is (PI - alpha_knee)
  // So, cos(q2) = -cos(alpha_knee)
  
  float cos_q2 = (L_hip_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  // Check for impossible positions (leg over-extended)
  if (cos_q2 > 1.0) {
    cos_q2 = 1.0;
  } else if (cos_q2 < -1.0) {
    cos_q2 = -1.0;
  }

  // This gives the relative knee angle.
  // We take the "elbow-back" (or knee-forward) solution.
  q.q2 = acos(cos_q2);

  // 3. Calculate the absolute hip angle (q1)
  
  // Angle of the hip vector from the *vertical* Y-axis
  float beta = atan2(x_d, y_d);
  
  // Angle of the hip vector relative to the thigh (L1)
  // using Law of Cosines on the same triangle
  // L2^2 = L1^2 + L_hip^2 - 2*L1*L_hip*cos(alpha_hip)
  float cos_alpha_hip = (L1 * L1 + L_hip_sq - L2 * L2) / (2 * L1 * L_hip);

  // Clamp for safety
  if (cos_alpha_hip > 1.0) {
    cos_alpha_hip = 1.0;
  } else if (cos_alpha_hip < -1.0) {
    cos_alpha_hip = -1.0;
  }
  
  float alpha_hip = acos(cos_alpha_hip);

  // The final hip angle is the sum/difference of these two angles
  // For the standard "knee-forward" pose, we subtract.
  q.q1 = beta - alpha_hip;
  
  // q.q1 is now the absolute angle of the thigh (L1) from the vertical
  // q.q2 is now the relative angle of the shank (L2) from the thigh
  
  return q;
}