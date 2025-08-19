#pragma once

#include "Task_4_motor.hpp"
#include "Task_4_DualEncoder.hpp"
#include "Task_4_EncoderOdometry.hpp"

#include <math.h>

#define REDUPWM 5
#define MINPWM 30

namespace mtrn3100 {

class PIDController {
public:
    // constructor
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    // convert from microseconds to seconds
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        // Check division by zero
        if (dt == 0) {
            dt = 1e-6; 
        }

        // error = setpoint - (input - zero_ref);
        error = setpoint - input;


        // Proportional term
        float p_out = kp * error;

        // Integral term
        integral += error * dt;
        float i_out = ki * integral;

        // Derivative term
        derivative = (error - prev_error) / dt;
        float d_out = kd * derivative;

        // Combine
        output = p_out + i_out + d_out;

        prev_error = error;

        int signCheck = output < 0 ? -1 : 1;

        return signCheck * (output / REDUPWM < MINPWM ? MINPWM : output / REDUPWM);
    }

    float getError() {
        return error;
    }

    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void zeroTarget(float zero, float target) {
        // Reset integral, start new movement
        integral = 0;
        prev_error = 0;
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
    }

    void newTarget(float target) {
        // Reset integral, start new movement
        integral = 0;
        prev_error = 0;
        prev_time = micros();
        //zero_ref = zero;
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