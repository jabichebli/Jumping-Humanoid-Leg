// =========================================================================
// JumpingLeg.ino
// =========================================================================
// This is the main file. It runs the state machine based on sensor data.
// =========================================================================

#include <Servo.h>
#include <Wire.h> // Standard for I2C communication
#include "MPU6050_tockn.h" 

// Include all the custom parts of our project
#include "Config.h"
#include "Sensors.h"
#include "Events.h"
#include "Kinematics.h"
#include "Trajectory.h"

// --- Global Objects ---
Servo servo_hip;
Servo servo_knee;
MPU6050 mpu6050(Wire);

// --- State Machine ---
enum RobotState {
  STABILIZING, // On the ground, holding a pose
  JUMPING,     // Executing the jump trajectory
  FLIGHT,      // In the air
  LANDED       // Just hit the ground, absorbing impact
};
RobotState current_state = STABILIZING;

// --- Global Variables ---
SensorData current_sensor_data;     // Holds all current sensor readings
JointAngles q_desired;             // The desired [q1, q2] for the servos
unsigned long last_control_update = 0; // For our non-blocking timer
unsigned long jump_start_time = 0;     // To track time `t` during the jump
unsigned long landed_time = 0;       // To time the "LANDED" state

// =========================================================================
//  SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- Attach Servos ---
  servo_hip.attach(SERVO_HIP_PIN);
  servo_knee.attach(SERVO_KNEE_PIN);

  // --- Initialize Sensors ---
  // Pass the mpu6050 object to our sensor setup function
  setupSensors(mpu6050);

  // Start in a stable pose
  current_state = STABILIZING;
  Serial.println("Setup complete. Leg is STABILIZING.");
  last_control_update = micros();
}

// =========================================================================
//  MAIN LOOP
// =========================================================================
void loop() {
  // Run the controller at a fixed rate (e.g., 100 Hz)
  unsigned long now = micros();
  if (now - last_control_update >= CONTROL_LOOP_PERIOD_MICROS) {
    last_control_update = now;

    // This is the main function that does all the work
    runControlCycle();
  }
}

// =========================================================================
//  This function is called at 100 Hz
// =========================================================================
void runControlCycle() {
  
  // 1. READ SENSORS
  current_sensor_data = readSensors(mpu6050);

  // 2. RUN STATE MACHINE 
  switch (current_state) {
    
    case STABILIZING:
      // Run the stabilizing controller
      q_desired = getStabilizingAngles(); 
      
      // --- Check for transition ---
      // For now, we'll start a jump by sending 'j' in the Serial Monitor
      if (Serial.available() > 0) {
        if (Serial.read() == 'j') {
          Serial.println("State -> JUMPING");
          current_state = JUMPING;
          jump_start_time = micros(); // Start the jump timer!
        }
      }
      break;

    case JUMPING:
      // This is the main trajectory controller
      float t = (micros() - jump_start_time) / 1000000.0; // time in seconds
      q_desired = getJumpingAngles(t);

      // --- Check for transition ---
      // Also check if the trajectory time has finished
      if (checkTakeoff(current_sensor_data) || t > (T_HOLD + T_JUMP)) {
        Serial.println("State -> FLIGHT");
        current_state = FLIGHT;
      }
      break;

    case FLIGHT:
      // In flight, pull legs into a "tucked" position
      q_desired = getFlightAngles();

      // --- Check for transition ---
      if (checkTouchdown(current_sensor_data)) {
        Serial.println("State -> LANDED");
        current_state = LANDED;
        landed_time = millis(); // Start a timer for the landed state
      }
      break;

    case LANDED:
      // Absorb the impact 
      q_desired = getStabilizingAngles(); // Use same logic as STABILIZING

      // --- Check for transition ---
      // Hold this state for a short time (e.g., 1 sec) then go back to STABILIZING
      if (millis() - landed_time > 1000) {
        Serial.println("State -> STABILIZING");
        current_state = STABILIZING;
      }
      break;
  }

  // 3. WRITE TO ACTUATORS (Servos)
  // Convert our kinematic angles (in radians) to servo angles (0-270)
  int hip_servo_angle = mapAngleToServo(q_desired.q1, HIP_SERVO_MIN, HIP_SERVO_MAX, HIP_RADIAN_MIN, HIP_RADIAN_MAX);
  int knee_servo_angle = mapAngleToServo(q_desired.q2, KNEE_SERVO_MIN, KNEE_SERVO_MAX, KNEE_RADIAN_MIN, KNEE_RADIAN_MAX);

  servo_hip.write(hip_servo_angle);
  servo_knee.write(knee_servo_angle);

  // 4. (Optional) Print debug data
  // Serial.print("State: "); Serial.print(current_state);
  // Serial.print(" q1_d: "); Serial.print(q_desired.q1);
  // Serial.print(" q2_d: "); Serial.println(q_desired.q2);
}


// --- Helper function to map radians to servo MICROSECONDS ---
// This correctly maps the kinematic angle (rad) to the pulse width (us)
// for the 270-degree servos.
int mapAngleToServo(float rad, int servo_min_us, int servo_max_us, float rad_min, float rad_max) {
  // Use map(), but for floats
  float pulse_width = (rad - rad_min) * (servo_max_us - servo_min_us) / (rad_max - rad_min) + servo_min_us;
  return constrain((int)pulse_width, servo_min_us, servo_max_us);
}