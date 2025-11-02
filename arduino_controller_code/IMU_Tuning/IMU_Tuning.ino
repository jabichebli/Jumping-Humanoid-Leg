#include <Wire.h>
#include "MPU6050_tockn.h"

#include "Sensors.h"
#include "Sensors.cpp"

MPU6050 mpu6050(Wire);
unsigned long last_print = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // This calls the setup function from your Sensors.cpp
  setupSensors(mpu6050);
}

void loop() {
  // This calls the read function from your Sensors.cpp
  SensorData data = readSensors(mpu6050);

  // Print data at 10 Hz (100ms)
  if (millis() - last_print > 100) {
    last_print = millis();
    
    // 1. VERIFY ACCELERATION:
    // With the leg UPRIGHT and STILL, this should be ~1.0 G
    Serial.print("Accel_Z (G's): ");
    Serial.print(data.accel_z);

    // 2. VERIFY ANGLE:
    // With the leg UPRIGHT, this should be ~0.0 rad
    Serial.print("  |  Angle_q3 (rad): ");
    Serial.println(data.angle_q3);
  }
}