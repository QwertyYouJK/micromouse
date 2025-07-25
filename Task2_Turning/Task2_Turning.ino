#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"
#include "Controller.hpp"
#include "IMUOdometry.hpp"

#include "Wire.h"
#include <MPU6050_light.h>

// unsigned long timer = 0;
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

// for turning:
#define TKP 50
#define TKI 1
#define TKD 2
#define ABOUND 0.01 // error, radians

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHRAD, AXLEN);
mtrn3100::IMUOdometry IMU;

mtrn3100::PIDController turnPid(TKP, TKI, TKD);
mtrn3100::PIDController leftPid(MKP, MKI, MKD);
mtrn3100::PIDController rightPid(MKP, MKI, MKD);

mtrn3100::Controller controller(&encoder, &encoder_odometry, &leftMotor, &rightMotor, &leftPid, &rightPid, &turnPid);

float original_yaw = IMU.get_yaw(); // Get original yaw


void setup() {
  Serial.begin(9600);
  // IMU setup
  Serial.println("Starting up!");
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
  delay(1000);
  Serial.println("Done");

  // 90 degree CW turn
  controller.turnOdom(-90);
  controller.turnOdom(0.001);
  IMU.update();
  original_yaw = IMU.get_yaw(); // Get original yaw
}

void loop() {
  IMU.update();
  // IMU.update_xyz(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ() - 1.0);

  // Serial.print("original "); Serial.println(IMU.get_yaw());

  // if (IMU.get_accZ() - 1 >= 0.35) {
  //   float original_yaw = IMU.get_yaw(); // Get original yaw
  //   Serial.print("lifted "); Serial.println(original_yaw);
  //   // Task2_lifted = true;
    
  //   // wait for robot to come on ground
  //   unsigned long startTime = millis();
  //   while (millis() - startTime < 5000) {
  //     // Serial.print("stored original "); Serial.println(original_yaw);
  //     IMU.update();
  //   }
  //   // delay(5000);    
    IMU.update();
    float curr_yaw = IMU.get_yaw(); // get updated yaw
  //   // Serial.print("curr  "); Serial.println(curr_yaw);
    float difference = original_yaw - curr_yaw;
    // if (difference < 0) {
    //   difference -= 10;
    // } else {
    //   difference += 10;
    // }

    // controller.turnOdom(difference);
    // controller.turnOdom(0.001);
    Serial.print("curr  "); Serial.print(curr_yaw); Serial.print("orig  "); Serial.println(original_yaw);
    
    if (abs(difference) >= 1) {
      IMU.update();
      if (difference < 0) {
        leftMotor.setPWM(-30);
        rightMotor.setPWM(-30);
      } else {
        leftMotor.setPWM(30);
        rightMotor.setPWM(30);
      }
      // curr_yaw = IMU.get_yaw(); // get updated yaw
      // difference = original_yaw - curr_yaw;
    } else {
      leftMotor.setPWM(0);
      rightMotor.setPWM(0);
    }
      

  // }
  
  // small loop delay
  delay(50);
}
