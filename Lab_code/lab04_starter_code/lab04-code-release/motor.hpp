#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        Serial.begin(115200);
        // TODO: Set both pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {

      // TODO: Output digital direction pin based on if input signal is positive or negative.
      if (pwm >= 0) {
        digitalWrite(dir_pin, HIGH);
      } else {
        digitalWrite(dir_pin, LOW);
      }

      // TODO: Output PWM signal between 0 - 255.
      int16_t finalPWM = abs(pwm);
      if (finalPWM > 255) {
        finalPWM = 255;
      }
      analogWrite(pwm_pin, finalPWM);

      Serial.print("Set pin ");
      Serial.print(pwm_pin);
      Serial.print(" to ");
      Serial.println(finalPWM);
    }

    // moves dist amount of mm
    void move(Motor left, Motor right, int16_t dist, int16_t pwm) {
      int16_t delay_time = dist / mm_per_delay;
      
      // 200mm = 1500 delay (100 PWM)
      left.setPWM(pwm * -1);
      right.setPWM(pwm);

      delay(delay_time);

      left.setPWM(0);
      right.setPWM(0);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    const uint16_t mm_per_delay = 0.133333333333333333;
};

}  // namespace mtrn3100
