#include "Task_4_motor.hpp"
#include "Task_4_DualEncoder.hpp"
#include "Task_4_EncoderOdometry.hpp"
#include "Task_4_PID_controller.hpp"
#include "Task_4_controller.hpp"
#include "Task_4_LIDAR.hpp"
#include "Task_4_IMUOdometry.hpp"
#include "Task_4.1.1.hpp"

#include "Wire.h"
#include <MPU6050_light.h>

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
#define ACPTPWM 100 // acceptable PWM
#define MOTOFF 0 // off
#define LEFTADJ -1 // adjustment values
#define RIGHTADJ 1 // adjustment values

// robot constants
#define WHRAD 22.5 // Wheel radius
#define AXLEN 103 // Axle length

// motor PID controller constants
#define MKP 100 // proportional gain
#define MKI 1 // integral gain
#define MKD 5 // derivative gain
#define MBOUND 2//2 // error, millimetres

// for turning:
#define TKP 100
#define TKI 0
#define TKD 0

#define TIKP 0
#define TIKI 0
#define TIKD 0

#define ABOUND 0.01 // error, radians

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
mtrn3100::IMUOdometry IMU;

mtrn3100::PIDController turnPid(TKP, TKI, TKD);
mtrn3100::PIDController leftPid(MKP, MKI, MKD);
mtrn3100::PIDController rightPid(MKP, MKI, MKD);

mtrn3100::Lidar leftLidar(SENSOR_LEFT);
mtrn3100::Lidar frontLidar (SENSOR_FRONT);
mtrn3100::Lidar rightLidar(SENSOR_RIGHT);

mtrn3100::Controller controller(
  &encoder, 
  &encoder_odometry, 
  &leftMotor, 
  &rightMotor, 
  &leftPid, 
  &rightPid, 
  &turnPid,
  frontLidar,
  leftLidar,
  rightLidar,
  &IMU
  );

int original_yaw;
String token;

String seq41 = "flfrffffffrflfrffffs";
char seq42[] = 
// "F178;F178;F182;F182;F178;F178;T90;F178;T-90;F182;T90;F178;T90;F182;T-90;F178;F178;F178;F178;T90;F178;F178;T86;f325;T-37;f118;T-24;f290;T16;f166;T37;f252;T-78;F178;T-90;F178;T90;F182;T-90;F178;F178;F178;T-90;F182;T90;F178;T-90;F178;T90;F178;T-90;F178;F182;T-90;F178;T-90;F182";
"F178;T-90;F178;T-90;F178;T85;f296;T-128;f172;T-18;f290;T18;f209;T41;f216;T-84;F178;T-90;F178;T90;F182;T-90;F178;T-90;F182;";

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up!");

  //////////////////// Lidar setup ////////////////////
  frontLidar.begin(ADDR_FRONT);
  Serial.print("Front LiDAR ready @0x");
  Serial.println(ADDR_FRONT, HEX);

  leftLidar.begin(ADDR_LEFT);
  Serial.print("Left LiDAR ready @0x");
  Serial.println(ADDR_LEFT, HEX);

  rightLidar.begin(ADDR_RIGHT);
  Serial.print("Right LiDAR ready @0x");
  Serial.println(ADDR_RIGHT, HEX);

  //////////////////// IMU setup ////////////////////
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets(true, true);
  delay(1000);
  Serial.println("Done");
  IMU.update();
  original_yaw = IMU.get_yaw(); // Store original heading yaw
  Serial.print("IMU ready, facing: ");
  Serial.println(original_yaw);

  // controller.sequence_move(seq41);
  // controller.execute_sequence(seq42);
}

void loop() {
  // Serial.print("leftDist  = ");
  // Serial.print(leftLidar.readMillimetres());
  // Serial.print(" rightDist  = ");
  // Serial.print(rightLidar.readMillimetres());
  // Serial.print(" frontDist  = ");
  // Serial.println(frontLidar.readMillimetres());
}
