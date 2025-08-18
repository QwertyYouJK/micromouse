#pragma once

#include <Arduino.h>
#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"
#include "LIDAR.hpp"

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

// PID
#define MBOUND 2 // error, millimetres

// control constants
#define TARGET_DIST 70 // mm desired gap
#define TOLERANCE 2 // mm deadband 
#define RAMP_STEP 12 // max PWM change per loop
#define KP_LIDAR 0.8 


namespace mtrn3100 {

class Controller {
public:
  Controller(
    DualEncoder* en,
    EncoderOdometry* enOdom,
    Motor* leftM,
    Motor* rightM,
    PIDController* leftPid,
    PIDController* rightPid,
    Lidar& frontLidar   // only this one is ever enabled
  ) :
    encoder(en),
    encoderOdometry(enOdom),
    leftMotor(leftM),
    rightMotor(rightM),
    leftPid(leftPid),
    rightPid(rightPid),
    lidar(frontLidar),
    lastPWM(0)
  {}

    void moveStraightOdom(float input) {
      float target = input / WHRAD;
      float startLeft = (WHRAD * encoder->getLeftRotation());
      float startRight = (WHRAD * encoder->getRightRotation());

      leftPid->zeroTarget(startLeft, startLeft + input);
      rightPid->zeroTarget(startRight, startRight + input);

      Serial.print("moving straight: ");
      Serial.print(input);
      Serial.println(" mm.");

      while(1) {
        float currLeft = (WHRAD * encoder->getLeftRotation());
        float currRight = (WHRAD * encoder->getRightRotation());
        Serial.println(currLeft);

        float outLeft = leftPid->compute(currLeft);
        float outRight = rightPid->compute(currRight);

        leftMotor->setPWM(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));
        rightMotor->setPWM(constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM));

        if (abs(leftPid->getError()) < MBOUND && abs(rightPid->getError()) < MBOUND) {
          break;
        }
      }

      leftMotor->setPWM(MOTOFF);
      rightMotor->setPWM(MOTOFF);
      
      delay(10);
      
    }

    // Positive means Counterclockwise, negative means CW
    void turnOdom(float myAngleDegrees) {
        encoderOdometry->update(encoder->getLeftRotation(),encoder->getRightRotation());
        float startAngle = encoderOdometry->getH(); //rad
        float targetAngle = startAngle + (myAngleDegrees * PI / 180);
        turnPid->newTarget(targetAngle);
        // should always be between -PI and +PI radians
        float flip = 1;
        if (myAngleDegrees <= -180 && myAngleDegrees < 0) {
            // right turn
            flip = -1;
        }

        while(1) {
            encoderOdometry->update(encoder->getLeftRotation(),encoder->getRightRotation());
            float currAngle = (encoderOdometry->getH());
            float turnPWM = turnPid->compute(currAngle);

            leftMotor->setPWM(constrain(-turnPWM * flip * LEFTADJ, -ACPTPWM, ACPTPWM));
            rightMotor->setPWM(constrain(turnPWM * flip * RIGHTADJ, -ACPTPWM, ACPTPWM));

            if (abs(turnPid->getError()) < ABOUND) {
                turnPid->newTarget(0);
                Serial.println("completed");
                break;
            }
        }

        leftMotor->setPWM(MOTOFF);
        rightMotor->setPWM(MOTOFF);
        delay(10);
        flip = 1;
    }

    /** Continuous P-control + ramp for front-facing wall follow */
    void followWallContinuous() {
        uint16_t dist = lidar.readMillimetres();          // only front sensor
        int16_t err = int(dist) - int(TARGET_DIST);
        Serial.println(err); 
        int16_t cmd = 0;
        // Move forward or backwards depending on tolerance
        if (err > TOLERANCE) {
          cmd = int16_t(KP_LIDAR * err);
        } else if (err < -TOLERANCE) {
          cmd = int16_t(KP_LIDAR * err);
        }
        int16_t delta = cmd - lastPWM;
        if (delta > RAMP_STEP) {
          delta = RAMP_STEP;
        } else if (delta < -RAMP_STEP) {
          delta = -RAMP_STEP;
        }
        lastPWM += delta;

        leftMotor->setPWM(constrain(lastPWM * LEFTADJ,  -MAXPWM, MAXPWM));
        rightMotor->setPWM(constrain(lastPWM * RIGHTADJ, -MAXPWM, MAXPWM));
    }


private:
  DualEncoder* encoder;
  EncoderOdometry* encoderOdometry;
  Motor* leftMotor;
  Motor* rightMotor;
  PIDController* leftPid;
  PIDController* rightPid;
  PIDController* turnPid;
  Lidar& lidar;
  int16_t lastPWM;
};

}