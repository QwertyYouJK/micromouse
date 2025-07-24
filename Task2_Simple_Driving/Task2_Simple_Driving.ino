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
#define TKP 5000
#define TKI 0
#define TKD 0
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

void setup() {
  Serial.begin(9600);
  // IMU setup
  Serial.println("Starting up!");
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
  delay(1000);
  Serial.println("Done");
}

void loop() {
  IMU.update();
  IMU.update_xyz(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ() - 1.0);

  float original_yaw = IMU.get_yaw(); // Get original yaw
  Serial.print("original "); Serial.println(original_yaw);

  if (IMU.get_accZ() - 1 >= 0.4) {
    Serial.println("Lifted");
    // Task2_lifted = true;
    
    // wait for robot to come on ground
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {
      IMU.update();
    }
    
    IMU.update();
    float curr_yaw = IMU.get_yaw(); // get updated yaw
    Serial.print("curr  "); Serial.println(curr_yaw);
    float difference = original_yaw - curr_yaw;
    Serial.println(difference);

    if (difference > 0) {
      // turn left
      leftMotor.setPWM(30);
      rightMotor.setPWM(30);
      delay(abs(difference * 100));
    } else {
      // turn right
      leftMotor.setPWM(-30);
      rightMotor.setPWM(-30);
      delay(abs(difference * 100));
    }
    // stop
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
  }
  
  // small loop delay
  delay(50);
}
