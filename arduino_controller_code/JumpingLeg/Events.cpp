// =========================================================================
// Events.cpp
// =========================================================================
#include "Events.h"
#include "Config.h" // For thresholds
#include <Arduino.h> // For abs()

// Checks if the leg is in freefall
bool checkTakeoff(const SensorData& data) {
  // Takeoff happens when vertical acceleration is near 0 (freefall)
  return (abs(data.accel_z) < TAKEOFF_THRESHOLD_G);
}

// Checks if the leg has hit the ground
bool checkTouchdown(const SensorData& data) {
  // Touchdown is detected by a large acceleration spike
  return (data.accel_z > LANDING_IMPACT_G);
}