#pragma once

#include "motor.hpp"
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"

#include <math.h>

#define REDUPWM 5
#define MINPWM 30

namespace mtrn3100 {

class PIDController {
public:
    // constructor
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    
    float compute(float input) {
      // convert from microseconds to seconds
      currTime = micros();
      dt = static_cast<float>(currTime - prevTime) / 1e6;
      prevTime = currTime;

      // Check division by zero
      if (dt == 0) {
          dt = 1e-6; 
      }

      error = setpoint - input;

      // Proportional term
      float p_out = kp * error;

      // Integral term
      integral += error * dt;
      float i_out = ki * integral;

      // Derivative term
      derivative = (error - prevError) / dt;
      float d_out = kd * derivative;

      // COmbine
      output = p_out + i_out + d_out;

      prevError = error;

      int signCheck = output < 0 ? -1 : 1;

      float finalOutput = output / REDUPWM;
      if (abs(finalOutput) < MINPWM) {
        finalOutput = (finalOutput >= 0) ? MINPWM : -MINPWM;
      }

      return finalOutput;
    }

    float getError() {
      return error;
    }

    void newTarget(float target) {
        // Reset integral, start new movement
        integral = 0;
        prevError = 0;
        prevTime = micros();
        //zero_ref = zero;
        setpoint = target;
    }


public:
    uint32_t prevTime, currTime = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prevError = 0;
    float setpoint = 0;
    //float zero_ref = 0;
};

}  // namespace mtrn3100