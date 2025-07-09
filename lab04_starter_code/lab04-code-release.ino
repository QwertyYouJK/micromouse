#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IMUOdometry.hpp"
#include "motor.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define MOTLPWM 11
#define MOTLDIR 12

#define MOTRPWM 9 // PIN 9 is a PWM pin
#define MOTRDIR 10

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(16,90); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH
mtrn3100::IMUOdometry IMU_odometry;

// set up the motors
mtrn3100::Motor leftMotor(MOTLPWM, MOTLDIR);
mtrn3100::Motor rightMotor(MOTRPWM, MOTRDIR);

bool forward = true;
bool rest = false;
bool backward = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
    //explode();
}


void loop() {

//UNCOMMENT FOR TASK 2: 
//THE DELAY IS REQUIRED OTHERWISE THE ENCODER DIFFERENCE IS TOO SMALL
    //delay(50);
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

//UNCOMMET FOR TASK 3:
//NOTE: IMU ODOMETRY IS REALLY BAD, THIS TASK EXISTS TO TEACH YOU WHY IMU ODOMETRY SUCKS, DO NOT SPEND TOO LONG ON IT
    //mpu.update();
    //IMU_odometry.update(mpu.getAccX(),mpu.getAccY());
    

    //Serial.print("ODOM:\t\t");
    //Serial.print(encoder_odometry.getX());
    //Serial.print(",\t\t");
    //Serial.print(encoder_odometry.getY());
    //Serial.print(",\t\t");
    //Serial.print(encoder_odometry.getH());
    //Serial.println();

// TASK 4
    if (forward) {
        Serial.println("Moving forward...");
        // left motor
        leftMotor.setPWM(-100);
        // right motor
        rightMotor.setPWM(100);

        // check if 2 rotations
        const float goal = 64 * PI;
        if (encoder_odometry.getX() == goal) {
            forward = false;
            rest = true;
        }
    } else if (rest) {
        Serial.println("Resting...");
        // left motor
        leftMotor.setPWM(0);
        // right motor
        rightMotor.setPWM(0);
        delay(2000);
        rest = false;
        backward = true;
    } else if (backward) {
        Serial.println("Moving backward...");
        // left motor
        leftMotor.setPWM(100);
        // right motor
        rightMotor.setPWM(-100);

        // check if 2 rotations
        const float goal = 64 * PI;
        if (encoder_odometry.getX() == goal) {
            forward = false;
            rest = false;
        }
    } else {
        Serial.println("Program Complete");
        delay(99999);
    }
}
