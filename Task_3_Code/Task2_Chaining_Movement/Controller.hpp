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
#define MBOUND 3 // error, millimetres
#define ABOUND 0.01 // error, radians

namespace mtrn3100 {

class Controller {
public:
    Controller(DualEncoder* en,
               EncoderOdometry* enOdom,
               Motor* leftM,
               Motor* rightM,
               PIDController* leftPid,
               PIDController* rightPid,
               PIDController* turnPid
            ): 
    encoder(en),
    encoderOdometry(enOdom),
    leftMotor(leftM),
    rightMotor(rightM),
    leftPid(leftPid),
    rightPid(rightPid),
    lidar(frontLidar),
    lastPWM(0)
    {}

    
    // (in mm) +ve move forward, -ve backward
    void moveStraightOdom(float input) {
        float startLeft = (WHRAD * encoder->getLeftRotation());
        float startRight = (WHRAD * encoder->getRightRotation());

        leftPid->newTarget(startLeft + input);
        rightPid->newTarget(startRight + input);

        while(1) {
            float currLeft = (WHRAD * encoder->getLeftRotation());
            float currRight = (WHRAD * encoder->getRightRotation());

            float outLeft = leftPid->compute(currLeft);
            float outRight = rightPid->compute(currRight);

            leftMotor->setPWM(constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM));
            rightMotor->setPWM(constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM));

            if (abs(leftPid->getError()) < MBOUND && abs(rightPid->getError()) < MBOUND) {
                leftPid->newTarget(0);
                rightPid->newTarget(0);
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
        Serial.println(startAngle);
        Serial.println(targetAngle);
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
            Serial.print("current Angle: ");
            Serial.println(currAngle);
            leftMotor->setPWM(constrain(-turnPWM * flip * LEFTADJ, -ACPTPWM, ACPTPWM));
            rightMotor->setPWM(constrain(turnPWM * flip * RIGHTADJ, -ACPTPWM, ACPTPWM));
            Serial.print("error: ");
            Serial.println(turnPid->getError());
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

private:
    DualEncoder* encoder;
    EncoderOdometry* encoderOdometry;
    Motor* leftMotor;
    Motor* rightMotor;
    PIDController* leftPid;
    PIDController* rightPid;
    PIDController* turnPid;
};

}