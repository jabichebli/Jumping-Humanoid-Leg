// =========================================================================
// Sensors.cpp
// =========================================================================
#include "Sensors.h"
#include <Arduino.h>

// This function is called once in setup()
void setupSensors(MPU6050& mpu) {
  Serial.println("Initializing MPU-6050...");
  mpu.begin();

  // === Calibrate Gyro ===
  // This is a blocking call, so make sure the leg is
  // PERFECTLY STILL on startup.
  Serial.println("Calibrating Gyro... Do not move!");
  mpu.calcGyroOffsets();
  
  // === Set Accelerometer Range ===
  // We set a higher range (8G) to make sure we can
  // accurately measure the high-G impact on landing.
  // Options are: MPU6050_ACC_RANGE_2G, MPU6050_ACC_RANGE_4G,
  // MPU6050_ACC_RANGE_8G, MPU6050_ACC_RANGE_16G
  mpu.setAccFullScaleRange(MPU6050_ACC_RANGE_8G);
  
  Serial.println("IMU setup complete.");
}


// This function is called every loop (at 100 Hz)
SensorData readSensors(MPU6050& mpu) {
  SensorData data;

  // --- 1. Update IMU data ---
  // This function must be called every loop to get new values
  mpu.update();

  // --- 2. Read Vertical Acceleration ---
  // We need to figure out which axis (X, Y, or Z) is vertical
  // based on how you mount the IMU.
  // Let's assume the Z-axis is vertical.
  //
  // TO-DO: Verify this! Hold the leg still and upright.
  // The Serial Monitor should print a value close to 1.0 G.
  // If it's 0, try getAccX() or getAccY().
  // If it's -1.0, just flip the sign.
  //
  // getAccZ() returns m/s^2. We divide by 9.81 to get G's.
  data.accel_z = mpu.getAccZ() / 9.81;

  // --- 3. Read Body Angle (q3) ---
  // We need to figure out which axis is the "pitch" axis.
  //
  // TO-DO: Verify this! Rock the leg back and forth
  // and see which angle value changes as you expect.
  //
  // We convert from degrees (library default) to radians.
  data.angle_q3 = mpu.getAngleY() * (PI / 180.0);
  
  return data;
}