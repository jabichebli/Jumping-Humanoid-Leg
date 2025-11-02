#include <Servo.h>

// --- TO-DO: Change these pins to match your setup ---
const int SERVO_PIN = 9; // Test one servo at a time (e.g., hip)

Servo test_servo;
int pulse_width = 1500; // 1500us is the neutral/center pulse

void setup() {
  Serial.begin(115200);
  test_servo.attach(SERVO_PIN);
  
  Serial.println("Servo Calibration Sketch");
  Serial.println("Enter a microsecond value (e.g., 500) in the Serial Monitor.");
  
  // Go to center position on startup
  test_servo.writeMicroseconds(pulse_width);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the integer from the serial monitor
    int new_pulse = Serial.parseInt();
    
    if (new_pulse > 300 && new_pulse < 2700) { // Safety check
      pulse_width = new_pulse;
      test_servo.writeMicroseconds(pulse_width);
      Serial.print("Commanding: ");
      Serial.println(pulse_width);
    }
  }
}