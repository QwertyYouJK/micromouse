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
#define MBOUND 4 // error, millimetres
#define ABOUND 0.01 // error, radians
#define IMU_ABOUND 1 // degrees, error

// control constants
#define TARGET_DIST 70 // mm desired gap
#define TOLERANCE 2 // mm deadband 
#define RAMP_STEP 12 // max PWM change per loop
#define KP_LIDAR 0.8 

#define MIN_WALL_DIST 40.0   // mm
#define MIN_FRONT_WALL_DIST 50.0   // mm
#define CORR_GAIN 0.4    // tuning factor

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
      float dist_gain = 1.065; 
      float targetLeft = ((WHRAD * encoder->getLeftRotation() + input) * dist_gain);
      float targetRight = ((WHRAD * encoder->getRightRotation() + input) * dist_gain);

      leftPid->newTarget(targetLeft);
      rightPid->newTarget(targetRight);

      Serial.print("moving straight: ");
      Serial.print(input);
      Serial.println(" mm.");

      while(1) {
        float currLeft = (WHRAD * encoder->getLeftRotation());
        float currRight = (WHRAD * encoder->getRightRotation());
        
        Serial.println(currLeft);

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

      uint16_t leftDist  = leftLidar.readMillimetres();
      uint16_t rightDist = rightLidar.readMillimetres();
      uint16_t frontDist = frontLidar.readMillimetres();

      // Stop early if front wall too close
      if (frontDist < MIN_FRONT_WALL_DIST && !frontLidar.timeoutOccurred()) {
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

  void moveCorrectionStraightOdomAvg(float input) {
    // Use average encoder distance as the main forward measure
    float startLeft = WHRAD * encoder->getLeftRotation();
    float startRight = WHRAD * encoder->getRightRotation();
    float startAvg = 0.5f * (startLeft + startRight);

    float targetAvg = startAvg + input;

      leftPid->newTarget(targetAvg);
      rightPid->newTarget(targetAvg);

      // get initial angle to compare to the end for adjustments
      // IMU->update();
      // float init_heading = IMU->get_yaw();

      // encoderOdometry->update(encoder->getLeftRotation(),encoder->getRightRotation());
      // float init_heading = encoderOdometry->getH();

      // Serial.print("init: ");
      // Serial.println(init_heading);

      Serial.print("moving straight: ");
      Serial.print(input);
      Serial.println(" mm.");

      while (1) {
        IMU->update();
        float currLeft = WHRAD * encoder->getLeftRotation();
        float currRight = WHRAD * encoder->getRightRotation();
        float currAvg = 0.5f * (currLeft + currRight);

        // PID now tracks average forward displacement
        float outLeft = leftPid->compute(currAvg);
        float outRight = rightPid->compute(currAvg);

        float leftPWM = constrain(outLeft * LEFTADJ, -ACPTPWM, ACPTPWM);
        float rightPWM = constrain(outRight * RIGHTADJ, -ACPTPWM, ACPTPWM);

        uint16_t leftDist  = leftLidar.readMillimetres();
        uint16_t rightDist = rightLidar.readMillimetres();
        uint16_t frontDist = frontLidar.readMillimetres();
        // ---------------------------
        // LIDAR wall correction
        // ---------------------------

        IMU->update();

        // // Stop early if front wall too close
        // if (frontDist < MIN_FRONT_WALL_DIST && !frontLidar.timeoutOccurred()) {
        //     Serial.println("Front wall detected - stopping early.");
        //     IMU->update();
        //     break;
        // }

        // If left wall close, push robot slightly right
        if (leftDist < MIN_WALL_DIST && !leftLidar.timeoutOccurred()) {
          float corr = CORR_GAIN * (MIN_WALL_DIST - leftDist);
          leftPWM += corr;
          rightPWM += corr;
          IMU->update();
        }

        // If right wall close, push robot slightly left
        if (rightDist < MIN_WALL_DIST && !rightLidar.timeoutOccurred()) {
          float corr = CORR_GAIN * (MIN_WALL_DIST - rightDist);
          leftPWM -= corr;
          rightPWM -= corr;
          IMU->update();
        }

        leftMotor->setPWM(leftPWM);
        rightMotor->setPWM(rightPWM);

        // Exit if close enough to target
        if (fabs(targetAvg - currAvg) < MBOUND) {
          IMU->update();
          break;
        }
      }

      // // check heading
      // delay(10);
      
      // // encoderOdometry->update(encoder->getLeftRotation(),encoder->getRightRotation());
      // // float curr_heading = encoderOdometry->getH();
      // // float correction_angle = (init_heading - curr_heading) * (PI / 180);
      // IMU->update();
      // float curr_heading = IMU->get_yaw();
      // float correction_angle = (init_heading - curr_heading);

      // Serial.print(" curr: "); Serial.print(curr_heading);
      // Serial.print(" turned: "); Serial.println(correction_angle);

      // IMU->update();
      // //turnOdom(correction_angle);
      // turnIMU(correction_angle);
      // delay(10);

      // IMU->update();
      // float snap_to_angle = round(correction_angle / 90.0 ) * 90.0;
      // turnIMU(snap_to_angle);

      delay(10);
      leftMotor->setPWM(MOTOFF);
      rightMotor->setPWM(MOTOFF);
      delay(10);
    }

    // Positive means Counterclockwise, negative means CW
    void turnOdom(float myAngleDegrees) {
        float turn_gain = 1.032;
        encoderOdometry->update(encoder->getLeftRotation(),encoder->getRightRotation());
        float startAngle = encoderOdometry->getH(); //rad
        float targetAngle = (startAngle + (myAngleDegrees * (PI / 180)) * turn_gain);
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

    void turnIMU(float my_angle_degrees) {
      IMU->update();
      float init_angle = IMU->get_yaw();
      float target_angle = init_angle + my_angle_degrees;

      turnPid->newTarget(target_angle);

      Serial.print("Turning with IMU to: ");
      Serial.println(target_angle);

      while(1) {
        IMU->update();
        float curr_angle = IMU->get_yaw();
        float turn_pwm = turnPid->compute(curr_angle);

        leftMotor->setPWM(constrain(-turn_pwm * LEFTADJ, -ACPTPWM, ACPTPWM));
        rightMotor->setPWM(constrain(turn_pwm * RIGHTADJ, -ACPTPWM, ACPTPWM));


        if (abs(turnPid->getError()) < IMU_ABOUND) {
          Serial.println("IMU turn finsished");
          break;
        }
      }

      leftMotor->setPWM(MOTOFF);
      rightMotor->setPWM(MOTOFF);
      delay(10);

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

      checkAndRetreat();
      switch(receivedChar) {
        case 'l':
          turnOdom(85);
          delay(100);
          break;
        case 'r':
          turnOdom(-85);
          delay(100);
          break;
        case 'f':
          moveStraightOdomAvg(300);
          delay(50);
          break;
        case 'b':
          moveStraightOdomAvg(-182);
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
    
    void move_direction(int dir) {
      switch (dir) {
        case 0: 
          Serial.println("Moving straight"); 
          break;
        case 1: 
          Serial.println("Turning Right"); 
          turnOdom(-90);
          delay(100);  
          break;
        case 2: 
          Serial.println("Turning aroud"); 
          turnOdom(180);
          break;
        case 3: 
          Serial.println("Turning Left"); 
          turnOdom(90);
          delay(100);  
          break;
      }
      moveStraightOdomAvg(182);
      delay(50);
    }

    int front_lidar_dist() {
      return frontLidar.readMillimetres();
    }
    int left_lidar_dist() {
      return leftLidar.readMillimetres();
    }
    int right_lidar_dist() {
      return rightLidar.readMillimetres();
    }

     /** 
     * Check if something is < 30mm in front. 
     * If so, move backwards until the front distance >= 50mm.
     */
    void checkAndRetreat() {
      const int TOO_CLOSE = 50;   // mm
      const int SAFE_DIST = 55;   // mm

      // Read current distance
      uint16_t frontDist = frontLidar.readMillimetres();
      if (!frontLidar.timeoutOccurred() && frontDist < TOO_CLOSE) {
        Serial.print("Obstacle detected at: ");
        Serial.print(frontDist);
        Serial.println(" mm. Retreating...");

        int step_back = frontDist - SAFE_DIST;
        // Step backwards using odometry
        moveCorrectionStraightOdomAvg(step_back);  
        delay(50); // small pause between steps
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

  void execute_sequence(char *seq) {
    for (char *tok = strtok(seq, ";"); tok; tok = strtok(NULL, ";")) {
      if (!*tok) continue;
      char op = tok[0];
      float val = atof(tok + 1);
      checkAndRetreat();
      IMU->update();
      if (op == 'F') moveStraightOdomAvg(val);
      else if (op == 'f') moveStraightOdomAvg(val);
      else if (op == 'T') turnOdom(val);
      else {
        Serial.print("Unknown token: "); Serial.println(tok);
      }
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