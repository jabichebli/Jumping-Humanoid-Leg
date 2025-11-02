// =========================================================================
// Events.h
// =========================================================================
#pragma once
#include "Sensors.h" // Needs sensor data structure

// --- Function Prototypes ---
bool checkTakeoff(const SensorData& data);
bool checkTouchdown(const SensorData& data);