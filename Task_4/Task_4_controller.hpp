#pragma once

#include <Arduino.h>
#include "Task_4_motor.hpp"
#include "Task_4_DualEncoder.hpp"
#include "Task_4_EncoderOdometry.hpp"
#include "Task_4_PID_controller.hpp"
#include "Task_4_lidar.hpp"
#include "Task_4_IMUOdometry.hpp"

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
    PIDController* turnPid,
    Lidar& frontLidar, 
    Lidar& leftLidar,
    Lidar& rightLidar,
    IMUOdometry* IMU
  ) :
    encoder(en),
    encoderOdometry(enOdom),
    leftMotor(leftM),
    rightMotor(rightM),
    leftPid(leftPid),
    rightPid(rightPid),
    turnPid(turnPid),
    frontLidar(frontLidar),
    leftLidar(leftLidar),
    rightLidar(rightLidar),
    IMU(IMU),
    lastPWM(0)
  {}

    void moveStraightOdom(float input) {
      // float target = input / WHRAD;
      // float startLeft = (WHRAD * encoder->getLeftRotation());
      // float startRight = (WHRAD * encoder->getRightRotation());

      // leftPid->zeroTarget(startLeft, startLeft + input);
      // rightPid->zeroTarget(startRight, startRight + input);

      float targetLeft = (WHRAD * encoder->getLeftRotation()) + input;
      float targetRight = (WHRAD * encoder->getRightRotation()) + input;

      leftPid->newTarget(targetLeft);
      rightPid->newTarget(targetRight);

      Serial.print("moving straight: ");
      Serial.print(input);
      Serial.println(" mm.");

      while(1) {
        float currLeft = (WHRAD * encoder->getLeftRotation());
        float currRight = (WHRAD * encoder->getRightRotation());
        // Serial.println(currLeft);

        float outLeft = leftPid->compute(currLeft);
        float outRight = rightPid->compute(currRight);

        float leftPWM = constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM);
        float rightPWM = constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM);

        leftMotor->setPWM(leftPWM);
        rightMotor->setPWM(rightPWM);

        if (abs(leftPid->getError()) < MBOUND && abs(rightPid->getError()) < MBOUND) {
          break;
        }
      }
      leftMotor->setPWM(MOTOFF);
      rightMotor->setPWM(MOTOFF);
      delay(10);   
    }

    void moveStraightOdomAvg(float input) {
      // Use average encoder distance as the main forward measure
      float startLeft = WHRAD * encoder->getLeftRotation();
      float startRight = WHRAD * encoder->getRightRotation();
      float startAvg = 0.5f * (startLeft + startRight);

      float targetAvg = startAvg + input;

      leftPid->newTarget(targetAvg);
      rightPid->newTarget(targetAvg);

      Serial.print("moving straight: ");
      Serial.print(input);
      Serial.println(" mm.");

      while (1) {
          float currLeft = WHRAD * encoder->getLeftRotation();
          float currRight = WHRAD * encoder->getRightRotation();
          float currAvg = 0.5f * (currLeft + currRight);

          // PID now tracks average forward displacement
          float outLeft = leftPid->compute(currAvg);
          float outRight = rightPid->compute(currAvg);

          float leftPWM = constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM);
          float rightPWM = constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM);

          // ---------------------------
          // LIDAR wall correction
          // ---------------------------
          const float MIN_WALL_DIST = 40.0;   // mm
          const float CORR_GAIN = 0.3;    // tuning factor

          uint16_t leftDist  = leftLidar.readMillimetres();
          uint16_t rightDist = rightLidar.readMillimetres();
          uint16_t frontDist = frontLidar.readMillimetres();

          // Stop early if front wall too close
          if (frontDist < MIN_WALL_DIST && !frontLidar.timeoutOccurred()) {
              Serial.println("Front wall detected - stopping early.");
              break;
          }

          // If left wall close, push robot slightly right
          if (leftDist < MIN_WALL_DIST && !leftLidar.timeoutOccurred()) {
              float corr = CORR_GAIN * (MIN_WALL_DIST - leftDist);
              leftPWM -= corr;
              rightPWM -= corr;
          }

          // If right wall close, push robot slightly left
          if (rightDist < MIN_WALL_DIST && !rightLidar.timeoutOccurred()) {
              float corr = CORR_GAIN * (MIN_WALL_DIST - rightDist);
              leftPWM += corr;
              rightPWM += corr;
          }

          leftMotor->setPWM(leftPWM);
          rightMotor->setPWM(rightPWM);

          // Exit if close enough to target
          if (fabs(targetAvg - currAvg) < MBOUND) {
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
        uint16_t dist = frontLidar.readMillimetres();          // only front sensor
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

    void sequence_move(String sequence) { 
      char receivedChar;
      bool newData = false;
      for (int i = 0; sequence[i] != '\0'; i++) {
        char receivedChar = sequence[i];
        Serial.print("Executing: ");
        Serial.println(receivedChar);

        switch(receivedChar) {
          case 'l':
            turnOdom(90);
            delay(100);
            break;
          case 'r':
            turnOdom(-90);
            delay(100);
            break;
          case 'f':
            // moveStraightOdom(180);
            moveStraightOdomAvg(180);
            delay(50);
            break;
          case 'b':
            moveStraightOdom(-180);
            delay(50);
            break;
          case 'd':
            turnOdom(0.001);
            break;
          case 's':
            leftMotor->setPWM(MOTOFF);
            rightMotor->setPWM(MOTOFF);
            break;
        }
      }
    }

    void turn_to_angle(int original_yaw) {
      IMU->update();
      float curr_yaw = IMU->get_yaw(); // get updated yaw
      float difference = original_yaw - curr_yaw; 
      Serial.print("curr  "); Serial.print(curr_yaw); Serial.print("orig  "); Serial.println(original_yaw);
      
      // Calc difference in yaws and move in the correct direction
      if (abs(difference) >= 1) {
          IMU->update();
          if (difference < 0) {
            leftMotor->setPWM(-30);
            rightMotor->setPWM(-30);
          } else {
            leftMotor->setPWM(30);
            rightMotor->setPWM(30);
          }
        } else {
          leftMotor->setPWM(MOTOFF);
          rightMotor->setPWM(MOTOFF);
        }
      }

private:
  DualEncoder* encoder;
  EncoderOdometry* encoderOdometry;
  Motor* leftMotor;
  Motor* rightMotor;
  PIDController* leftPid;
  PIDController* rightPid;
  PIDController* turnPid;
  Lidar& frontLidar;
  Lidar& leftLidar;
  Lidar& rightLidar;
  IMUOdometry* IMU;
  int16_t lastPWM;
};

}