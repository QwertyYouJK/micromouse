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
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
      
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT PID CONTROLLER
        integral = 0;
        derivative = 0;
        output = 0;

        prev_error = 0;

        int signCheck = output < 0 ? -1 : 1;

        return signCheck * (output / REDUPWM < MINPWM ? MINPWM : output / REDUPWM);
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    void zeroTarget(float zero, float target) {
        // Reset integral, start new movement
        integral = 0;
        prev_error = 0;
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;

    
};

}  // namespace mtrn3100
