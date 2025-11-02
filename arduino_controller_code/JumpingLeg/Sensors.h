// =========================================================================
// Sensors.h
// =========================================================================
#pragma once
#include <Servo.h>
#include "MPU6050_tockn.h" 

// A struct to hold all our sensor data
struct SensorData {
  float accel_z;  // Vertical acceleration in G's
  float angle_q3; // Body pitch angle (from IMU)
};

// --- Function Prototypes ---
void setupSensors(MPU6050& mpu); 
SensorData readSensors(MPU6050& mpu);