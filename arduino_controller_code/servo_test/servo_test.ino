#include <Servo.h>

Servo myServoKnee;
Servo myServoHip;

void setup() {
  myServoKnee.attach(9);
  myServoHip.attach(10);


  // // // // Neutral Position
  // myServoKnee.writeMicroseconds(1500);  // knee squat
  // myServoHip.writeMicroseconds(1500);   // hip squat
  // delay(1500);

  // // Squat Position
  // myServoKnee.writeMicroseconds(1600);  // knee squat
  // myServoHip.writeMicroseconds(1250);   // hip squat
  // delay(300);

  // // // Extend Position
  // myServoKnee.writeMicroseconds(1000);  // knee squat
  // myServoHip.writeMicroseconds(2000);   // hip squat
  // delay(200);

  // // // // Neutral Position
  // myServoKnee.writeMicroseconds(1500);  // knee squat
  // myServoHip.writeMicroseconds(1500);   // hip squat
  // delay(1500);
}

void loop() {
  // Squat Position
  myServoKnee.writeMicroseconds(1600);  // knee squat
  myServoHip.writeMicroseconds(1250);   // hip squat
  delay(200);

  // // Extend Position
  myServoKnee.writeMicroseconds(1000);  // knee squat
  myServoHip.writeMicroseconds(2000);   // hip squat
  delay(200);

  // // // Neutral Position
  myServoKnee.writeMicroseconds(1500);  // knee squat
  myServoHip.writeMicroseconds(1500);   // hip squat
  delay(1500);}

