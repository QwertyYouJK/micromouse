#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
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
// todo
#define MAXPWM 255
#define MOTPWM 30
#define MOTOFF 0
#define LEFTADJ -1
#define RIGHTADJ 1

// define motor classes
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16, 90);  //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
// mtrn3100::IMUOdometry IMU_odometry;

bool move = true;

void setup() {
  Serial.begin(9600);
  // Wire.begin();
  //Set up the IMU
  // byte status = mpu.begin();
  // Serial.print(F("MPU6050 status: "));
  // Serial.println(status);
  // while (status != 0) {}  // stop everything if could not connect to MPU6050

  // Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(3000);
  // mpu.calcOffsets(true, true);
  // Serial.println("Done!\n");
}

void loop() {

  encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

  Serial.print("ODOM:\t\t");
  Serial.print(encoder_odometry.getX());
  Serial.print(",\t\t");
  Serial.print(encoder_odometry.getY());
  Serial.print(",\t\t");
  Serial.print(encoder_odometry.getH());
  Serial.println();

  // basic movement
  if (move) {
    leftMotor.setPWM(MOTPWM * LEFTADJ);
    rightMotor.setPWM(MOTPWM * RIGHTADJ);

    if (encoder_odometry.getX() >= 100) {
      move = false;
    }
  } else {
    // delay(500);
    leftMotor.setPWM(MOTOFF);
    rightMotor.setPWM(MOTOFF);    
  }

  
}
