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
    Controller(const DualEncoder& en, 
               const Motor& leftM,
               const Motor& rightM,
               const PIDController& leftPid,
               const PIDController& rightPid)
        : encoder(en)
        , leftMotor(leftM)
        , rightMotor(rightM)
        , leftPid(leftPid)
        , rightPid(rightPid)
    {}

    void moveStraightOdom(float input, bool moveCheck) {
        float target = input / WHRAD;
        float startLeft = (WHRAD * encoder.getLeftRotation());
        float startRight = (WHRAD * encoder.getRightRotation());

        leftPid.zeroTarget(startLeft, startLeft + input);
        rightPid.zeroTarget(startRight, startRight + input);

        Serial.print("moving straight: ");
        Serial.print(input);
        Serial.println(" mm.");

        while(moveCheck) {
        float currLeft = (WHRAD * encoder.getLeftRotation());
        float currRight = (WHRAD * encoder.getRightRotation());

        float outLeft = leftPid.compute(currLeft);
        float outRight = rightPid.compute(currRight);

        leftMotor.setPWM(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));
        rightMotor.setPWM(constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM));

        if (abs(leftPid.getError()) < MBOUND && abs(rightPid.getError()) < MBOUND) {
            moveCheck = false;
        }
        }

        leftMotor.setPWM(MOTOFF);
        rightMotor.setPWM(MOTOFF);

        delay(10);

    }

private:
    DualEncoder encoder;
    Motor leftMotor;
    Motor rightMotor;
    PIDController leftPid;
    PIDController rightPid;
};

}