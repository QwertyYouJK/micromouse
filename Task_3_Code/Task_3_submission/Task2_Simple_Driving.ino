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
#define ACPTPWM 200 // acceptable PWM
#define MOTOFF 0 // off
#define LEFTADJ -1 // adjustment values
#define RIGHTADJ 1 // adjustment values

// robot constants
#define WHRAD 24 // 22.5 // Wheel radius
#define AXLEN 102.5 // Axle length

// motor PID controller constants
#define MKP 50 // proportional gain
#define MKI 0 // integral gain
#define MKD 5 // derivative gain

// for turning:
#define TKP 5000
#define TKI 0
#define TKD 0

// straight movement, turning allowed error:
#define MBOUND 2 // error, millimetres
#define ABOUND 0.01 // error, radians

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoderOdometry(WHRAD, AXLEN);
// mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::PIDController turnPid(TKP, TKI, TKD);
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

void turnOdom(float myAngleDegrees, bool moveCheck) {
  encoderOdometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
  float startAngle = encoderOdometry.getH(); //rad
  float targetAngle = startAngle + (myAngleDegrees * PI / 180);
  turnPid.newTarget(targetAngle);
  // should always be between -PI and +PI radians
  float flip = 1;
  if (targetAngle <= -PI && targetAngle < 0) {
    // right turn
    flip = -1;
  }

  while(moveCheck) {
    encoderOdometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    float currAngle = (encoderOdometry.getH());
    float turnPWM = turnPid.compute(currAngle);

    leftMotor.setPWM(constrain(-turnPWM * flip * LEFTADJ, -ACPTPWM, ACPTPWM));
    rightMotor.setPWM(constrain(turnPWM * flip * RIGHTADJ, -ACPTPWM, ACPTPWM));

    if (abs(turnPid.getError()) < ABOUND) {
      moveCheck = false;
      Serial.println("completed");
    }
  }

  leftMotor.setPWM(MOTOFF);
  rightMotor.setPWM(MOTOFF);
  delay(10);
}


void loop() {
  bool allowMove = true;

  moveStraightOdom(200, allowMove);

  // delay(1000);

  // turnOdom(-90, allowMove);

  delay(999999999);

  // moveStraightOdom(-100, allowMove);

  // delay(1000);
  
}
