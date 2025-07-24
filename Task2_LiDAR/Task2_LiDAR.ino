#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"
#include "Controller.hpp"
#include "Wire.h"
#include "LIDAR.hpp"

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


#define SENSOR_LEFT A0
#define SENSOR_FRONT  A1
#define SENSOR_RIGHT A2

#define ADDR_FRONT 0x56
#define ADDR_LEFT  0x54
#define ADDR_RIGHT 0x55

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(WHRAD, AXLEN);
// mtrn3100::IMUOdometry IMU_odometry;

mtrn3100::PIDController leftPid(MKP, MKI, MKD);
mtrn3100::PIDController rightPid(MKP, MKI, MKD);
mtrn3100::Lidar leftLidar(SENSOR_LEFT);
mtrn3100::Lidar frontLidar (SENSOR_FRONT);
mtrn3100::Lidar rightLidar(SENSOR_RIGHT);
mtrn3100::Controller controller(&encoder, &encoder_odometry, &leftMotor, &rightMotor, &leftPid, &rightPid, frontLidar);


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // 1) Disable all three sensors
  frontLidar.disable();
  leftLidar.disable();
  rightLidar.disable();

  // 2) Enable & init only the front sensor
  frontLidar.begin(ADDR_FRONT);
  Serial.print("Front LiDAR ready @0x");
  Serial.println(ADDR_FRONT, HEX);

  // (we never call begin() on leftLidar or rightLidar)
}

void loop() {
  // Wall‐follow uses only the front sensor
  controller.followWallContinuous();
  delay(50);  // ~20 Hz
}
// 1 - left, 2 - front, 3 - right 
