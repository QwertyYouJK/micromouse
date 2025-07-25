#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"
#include "Controller.hpp"
#include "IMUOdometry.hpp"
#include "Keyboard.h"
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

void setup() {
  Serial.begin(9600);
  // IMU setup
  Serial.println("Starting up!");
  // Keyboard.begin();
  delay(2000);
  Serial.println("Done");
}

char receivedChar;
bool newData = false;
char sequence[] = "rdfdrdfdfdldldfds";

void loop() {
  for (int i = 0; sequence[i] != '\0'; i++) {
    char receivedChar = sequence[i];
    Serial.print("Executing: ");
    Serial.println(receivedChar);

    switch(receivedChar) {
      case 'l':
        controller.turnOdom(90);
        delay(50);
        break;
      case 'r':
        controller.turnOdom(-90);
        delay(100);
        break;
      case 'f':
        controller.moveStraightOdom(180);
        delay(50);
        break;
      case 'b':
        controller.moveStraightOdom(-180);
        delay(50);
        break;
      case 'd':
        controller.turnOdom(0.001);
        break;
      case 's':
        delay(1000000);
        break;
    }
  }
  

  // small loop delay
  delay(50);
}
