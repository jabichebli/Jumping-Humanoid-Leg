#include <Servo.h>
#include <Arduino.h> // For math functions

// =========================================================================
// --- 1. PASTE VALUES FROM Config.h ---
// (Copy your final, calibrated values here from Step 1)
// =========================================================================

// --- Hardware Pins ---
const int SERVO_HIP_PIN = 9;
const int SERVO_KNEE_PIN = 10;

// --- Physical Parameters (from MATLAB files) ---
const float L1 = 0.15; // length of thigh link (m)
const float L2 = 0.15; // length of shank link (m)

// --- Servo Calibration ---
// TO-DO: PUT YOUR CALIBRATED VALUES FROM STEP 1 HERE!
const int HIP_SERVO_MIN_US = 500;
const int HIP_SERVO_MAX_US = 2500;
const int KNEE_SERVO_MIN_US = 500;
const int KNEE_SERVO_MAX_US = 2500;

// TO-DO: PUT YOUR ANGLE MAPPING HERE!
// (e.g., if you only use 180 degrees)
const float HIP_RADIAN_MIN = 0.0;
const float HIP_RADIAN_MAX = 4.712; // 270 deg (3*PI / 2)
const float KNEE_RADIAN_MIN = 0.0;
const float KNEE_RADIAN_MAX = 4.712;

// --- Trajectory Parameters (from simulate_jumping_leg.m) ---
const float Y_STAND = 0.22;
const float Y_SQUAT = 0.06;
const float X_LEAN = 0.0;

// =========================================================================
// --- 2. PASTE CODE FROM OTHER FILES ---
// =========================================================================

// --- Struct from Kinematics.h ---
struct JointAngles { float q1; float q2; };

// --- Global Servos ---
Servo servo_hip;
Servo servo_knee;

// --- Function from JumpingLeg.ino ---
int mapAngleToServo(float rad, int servo_min_us, int servo_max_us, float rad_min, float rad_max) {
  float pulse_width = (rad - rad_min) * (servo_max_us - servo_min_us) / (rad_max - rad_min) + servo_min_us;
  return constrain((int)pulse_width, servo_min_us, servo_max_us);
}

// --- Function from Kinematics.cpp ---
JointAngles solveInverseKinematics(float x_d, float y_d) {
  JointAngles q;
  float L_hip_sq = x_d * x_d + y_d * y_d;
  float L_hip = sqrt(L_hip_sq);
  float cos_q2 = (L_hip_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (cos_q2 > 1.0) { cos_q2 = 1.0; } else if (cos_q2 < -1.0) { cos_q2 = -1.0; }
  q.q2 = acos(cos_q2);
  float beta = atan2(x_d, y_d);
  float cos_alpha_hip = (L1 * L1 + L_hip_sq - L2 * L2) / (2 * L1 * L_hip);
  if (cos_alpha_hip > 1.0) { cos_alpha_hip = 1.0; } else if (cos_alpha_hip < -1.0) { cos_alpha_hip = -1.0; }
  float alpha_hip = acos(cos_alpha_hip);
  q.q1 = beta - alpha_hip;
  return q;
}

// =========================================================================
// --- 3. MAIN TEST SKETCH ---
// =========================================================================

void setup() {
  Serial.begin(115200);
  servo_hip.attach(SERVO_HIP_PIN);
  servo_knee.attach(SERVO_KNEE_PIN);
  Serial.println("Kinematics Pose Test");
  Serial.println("Press 's' for stand, 'c' for crouch.");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 's') {
      Serial.println("Commanding STAND pose");
      // Get stable pose (x, y) from your Config.h values
      JointAngles q_stand = solveInverseKinematics(X_LEAN, Y_STAND);
      commandServos(q_stand);
      
    } else if (cmd == 'c') {
      Serial.println("Commanding SQUAT pose");
      // Get squat pose (x, y) from your Config.h values
      JointAngles q_crouch = solveInverseKinematics(X_LEAN, Y_SQUAT);
      commandServos(q_crouch);
    }
  }
}

void commandServos(JointAngles q) {
  int hip_pulse = mapAngleToServo(q.q1, HIP_SERVO_MIN_US, HIP_SERVO_MAX_US, HIP_RADIAN_MIN, HIP_RADIAN_MAX);
  int knee_pulse = mapAngleToServo(q.q2, KNEE_SERVO_MIN_US, KNEE_SERVO_MAX_US, KNEE_RADIAN_MIN, KNEE_RADIAN_MAX);
  
  Serial.print("  Angles (rad): q1="); Serial.print(q.q1); Serial.print(", q2="); Serial.println(q.q2);
  Serial.print("  Pulses (us): hip="); Serial.print(hip_pulse); Serial.print(", knee="); Serial.println(knee_pulse);

  servo_hip.writeMicroseconds(hip_pulse);
  servo_knee.writeMicroseconds(knee_pulse);
}