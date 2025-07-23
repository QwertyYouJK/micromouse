#pragma once

#include <Arduino.h>
#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "PIDcontroller.hpp"

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

namespace mtrn3100 {

class Controller {
public:
    Controller(DualEncoder* en,
               EncoderOdometry* enOdom,
               Motor* leftM,
               Motor* rightM,
               PIDController* leftPid,
               PIDController* rightPid)
        : encoder(en)
        , encoderOdometry(enOdom)
        , leftMotor(leftM)
        , rightMotor(rightM)
        , leftPid(leftPid)
        , rightPid(rightPid)
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

private:
    DualEncoder* encoder;
    EncoderOdometry* encoderOdometry;
    Motor* leftMotor;
    Motor* rightMotor;
    PIDController* leftPid;
    PIDController* rightPid;
};

}