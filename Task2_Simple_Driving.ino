#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"

#include "Wire.h"
// #include <MPU6050_light.h>

// MPU6050 mpu(Wire);

#define EN_1_A 2  //These are the pins for the PCB encoder
#define EN_1_B 7  //These are the pins for the PCB encoder
#define EN_2_A 3  //These are the pins for the PCB encoder
#define EN_2_B 8  //These are the pins for the PCB encoder

// define pins
#define MOTLPWM 11
#define MOTLDIR 12

#define MOTRPWM 9  // PIN 9 is a PWM pin
#define MOTRDIR 10

// define any other constants
// motor constants
#define MAXPWM 255 // max PWM
#define MOTPWM 30 // example PWM
#define ACPTPWM 50 // acceptable PWM
#define MOTOFF 0 // off
#define LEFTADJ -1 // adjustment values
#define RIGHTADJ 1 // adjustment values

// robot constants
#define WHRAD 22.5 // Wheel radius
#define AXLEN 103 // Axle length

// motor PID controller constants
#define MKP 8 // proportional gain
#define MKI 0.2 // integral gain
#define MKD 0.9 // derivative gain
#define MBOUND 2 // error, millimetres
#define ABOUND 0.1 // error, radians

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometry(WHRAD, AXLEN);
// mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::PIDController turnPid(MKP, MKI, MKD);
mtrn3100::PIDController leftPid(MKP, MKI, MKD);
mtrn3100::PIDController rightPid(MKP, MKI, MKD);

void setup() {
  Serial.begin(9600);
  Serial.println("ran!");
  delay(1000);
}

void moveStraightOdom(float input, bool moveCheck) {
  float startLeft = (WHRAD * encoder.getLeftRotation());
  float startRight = (WHRAD * encoder.getRightRotation());

  leftPid.newTarget(startLeft + input);
  rightPid.newTarget(startRight + input);

  while(moveCheck) {
    float currLeft = (WHRAD * encoder.getLeftRotation());
    float currRight = (WHRAD * encoder.getRightRotation());

    float outLeft = leftPid.compute(currLeft);
    float outRight = rightPid.compute(currRight);

    leftMotor.setPWM(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));
    rightMotor.setPWM(constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM));

    if (abs(leftPid.getError()) < MBOUND && abs(rightPid.getError()) < MBOUND) {
      moveCheck = false;
      leftPid.newTarget(0);
      rightPid.newTarget(0);
    }
  }

  leftMotor.setPWM(MOTOFF);
  rightMotor.setPWM(MOTOFF);
  delay(10);      
}

// void leftTurnOdom(bool moveCheck) {
//   encoderOdometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
//   float startAngle = encoderOdometry.getH();

//   turnPid.zeroTarget(startAngle, startAngle + PI/2);

//   Serial.print("turning left from: ");
//   Serial.print(startAngle);
//   Serial.print(" to: ");
//   Serial.println(startAngle + PI/2);

//   while(moveCheck) {
//     encoderOdometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
//     float currAngle = (encoderOdometry.getH());

//     float outLeft = leftPid.compute(-currAngle);
//     float outRight = rightPid.compute(currAngle);

//     leftMotor.setPWM(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));
//     rightMotor.setPWM(constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM));

//     Serial.print("speed set to: ");
//     Serial.println(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));

//     Serial.print("Left Error: ");
//     Serial.println(leftPid.getError());
//     Serial.print("Right Error: ");
//     Serial.println(rightPid.getError());

//     if (abs(leftPid.getError()) < ABOUND && abs(rightPid.getError()) < ABOUND) {
//       moveCheck = false;
//       Serial.println("completed");
//     }
//   }

//   leftMotor.setPWM(MOTOFF);
//   rightMotor.setPWM(MOTOFF);
//   delay(10);
// }
  
void rightTurnOdom(bool moveCheck) {

}

void loop() {
  bool allowMove = true;

  moveStraightOdom(100, allowMove);

  //leftTurnOdom(allowMove);

  delay(5000);

  moveStraightOdom(-100, allowMove);

  delay(5000);
  
}
