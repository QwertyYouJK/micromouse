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
#define MOTOFF 0 // off
#define LEFTADJ -1 // adjustment values
#define RIGHTADJ 1 // adjustment values

// robot constants
#define WHRAD 16 // Wheel radius
#define AXLEN 90 // Axle length

// PID controller constants
#define kP 12 // proportional gain
#define kI 12 //12 // integral gain
#define kD 12 //50 // derivative gain

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHRAD, AXLEN);  //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
// mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::PIDController pid(kP, kI, kD); // kp, ki, kd

bool allowMove = true;
int target = 100;

void setup() {
  Serial.begin(9600);
  // Wire.begin();
  //Set up the IMU
  // byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  // while (status != 0) {}  // stop everything if could not connect to MPU6050

  // Serial.println(F("Calculating offsets, do not move MPU6050"));
  pid.zeroAndTarget(0, 100);
  Serial.println("ran!");
  delay(3000);
  // mpu.calcOffsets(true, true);
  // Serial.println("Done!\n");

}

void loop() {

  encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

  // Serial.print("ODOM:\t\t");
  // Serial.print(encoder_odometry.getX());
  // Serial.print(",\t\t");
  // Serial.print(encoder_odometry.getY());
  // Serial.print(",\t\t");
  // Serial.print(encoder_odometry.getH());
  // Serial.println();

  // // basic movement
  // if (allowMove) {
  //   leftMotor.setPWM(MOTPWM * LEFTADJ);
  //   rightMotor.setPWM(MOTPWM * RIGHTADJ);

  //   if (encoder_odometry.getX() >= 100) {
  //     allowMove = false;
  //   }
  // } else {
  //   delay(500);
  //   leftMotor.setPWM(MOTOFF);
  //   rightMotor.setPWM(MOTOFF);    
  // }

  
  while (allowMove) {
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    float currentPos = encoder_odometry.getX();
    float raw_output = pid.compute(currentPos);
    float pwmSignal = constrain(raw_output, -50, 50);
    if (pwmSignal < 10 && pwmSignal > 0) {
      pwmSignal = 10;
    }
    if (pwmSignal > -10 && pwmSignal < 0) {
      pwmSignal = -10;
    }
    leftMotor.setPWM(pwmSignal * LEFTADJ);
    rightMotor.setPWM(pwmSignal * RIGHTADJ);
        
    Serial.print("Target: ");
    Serial.print(target);
    Serial.print(" | Current: ");
    Serial.print(currentPos);
    Serial.print(" | Error: ");
    Serial.print(pid.getError());
    Serial.print(" | PWM: ");
    Serial.println(pwmSignal);
    delay(10); // Small delay
    if (currentPos == target + 5 || currentPos == target - 5) {
      Serial.println("within range!");
      allowMove = false;
    }
  }

  leftMotor.setPWM(MOTOFF);
  rightMotor.setPWM(MOTOFF);


  
}
