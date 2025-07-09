#include "motor.hpp"

// define pins
#define MOTLPWM 11
#define MOTLDIR 12

#define MOTRPWM 9 // PIN 9 is a PWM pin
#define MOTRDIR 10

// define any other constants
// todo
#define MAXPWM 255
#define MOTPWM 100
#define MOTOFF 0
#define LEFTADJ -1
#define RIGHTADJ 1

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);


void setup() {
  Serial.begin(9600);
  // wait for 1 seconds
  delay(1000);
}

void loop() {
  
  // basic movement
  leftMotor.setPWM(MOTPWM * LEFTADJ);
  rightMotor.setPWM(MOTPWM * RIGHTADJ);

  // Serial.print("left motor set at: ");
  // Serial.print(MOTPWM * LEFTADJ);
  // Serial.print(" right motor set at: ");
  // Serial.println(MOTPWM * RIGHTADJ);

  delay(1500);

  // leftMotor.move(leftMotor, rightMotor, 200, 100);

  leftMotor.setPWM(MOTOFF);
  rightMotor.setPWM(MOTOFF);

  // Serial.print("left motor set at: ");
  // Serial.print(MOTOFF);
  // Serial.print(" right motor set at: ");
  // Serial.println(MOTOFF);

  delay(99999);

}
