// =========================================================================
// Config.h
// =========================================================================
// This file holds all global parameters and settings.
// =========================================================================

#pragma once // Prevents file from being included multiple times

// --- Control Loop ---
const int CONTROL_LOOP_PERIOD_MICROS = 10000; // 10000 us = 10 ms = 100 Hz

// --- Hardware Pins ---
const int SERVO_HIP_PIN = 9;
const int SERVO_KNEE_PIN = 10;

// --- Physical Parameters (from  MATLAB files) ---
const float L1 = 0.15; // length of thigh link (m)
const float L2 = 0.15; // length of shank link (m)

// --- Servo Calibration (TO-DO: Calibrate these!) ---
// NOTE: The standard servo.write(angle) command (0-180) will NOT give the full 270-degree range.
// We MUST use servo.writeMicroseconds() for full control.
// -------------------------------------------------------------------------
const int HIP_SERVO_MIN_US = 500;  // <<< TO-DO: Microseconds for 0 degrees
const int HIP_SERVO_MAX_US = 2500; // <<< TO-DO: Microseconds for 270 degrees
const int KNEE_SERVO_MIN_US = 500;
const int KNEE_SERVO_MAX_US = 2500;

// --- Kinematic Angle Mapping (TO-DO: Define your zero!) ---
// Define what 0-degrees on the servo *means* in your kinematic model.
// This requires careful assembly and measurement.
// Example: Your MATLAB model might define q1=0 as vertical, but your
// servo's 0-degree position might be horizontal.
//
// 0.0 radians = 0 degrees
// 4.712 radians = 270 degrees (3*PI / 2)
// -------------------------------------------------------------------------
const float HIP_RADIAN_MIN = 0.0; // <<< TO-DO: Kinematic angle (rad) for min servo pulse
const float HIP_RADIAN_MAX = 4.712; // <<< TO-DO: Kinematic angle (rad) for max servo pulse

const float KNEE_RADIAN_MIN = 0.0;
const float KNEE_RADIAN_MAX = 4.712;


// --- Trajectory Parameters (from simulate_jumping_leg.m) ---
const float T_HOLD = 0.6; 
const float T_JUMP = 0.7;  
const float Y_STAND = 0.22;
const float Y_SQUAT = 0.06;   
const float Y_TAKEOFF = 0.26;  
const float X_LEAN = 0.0; 
const float X_TAKEOFF = 0.0;  
const float T_DIP_RATIO = 0.75; 

// --- Event Detection (IMU) Thresholds (TO-DO: Tune these!) ---
// These are good starting values. They will need to be tuned by printing sensor data to the Serial Monitor.
const float TAKEOFF_THRESHOLD_G = 0.2;  // Triggers flight when |accel_z| < 0.2 Gs
const float LANDING_IMPACT_G = 3.0;     // Triggers landing when accel_z > 3.0 Gs

// --- Controller Setpoints (MAYBE TO-DO: Tune these!) ---
// The MATLAB code sets flight angles to the last takeoff angle.
// For hardware, it's easier to command a fixed "tucked" pose.
// -------------------------------------------------------------------------
const float FLIGHT_HIP_ANGLE_RAD = 0.8726646; // 50 deg at hip
const float FLIGHT_KNEE_ANGLE_RAD = 1.705658;  // 97.727 deg at knee