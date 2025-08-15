#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        instance = this;  // Store the instance pointer
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        // Attach the interrupt on pin one such that it calls the readEncoderISR function on a rising edge. 
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR, RISING);
    }

    // Encoder function used to update the encoder
    void readEncoder() {
        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // Increase or Decrease the count by one based on the reading on encoder pin 2.
        if (digitalRead(encoder2_pin) == LOW) {
            count--;
        } else {
            count++;
        }
    }

    // Helper function which to convert encoder count to radians.
    float getRotation() {

        // Convert encoder count to radians.
        if (counts_per_revolution == 0) {
            return 0; // Avoid division by zero.
        }
        return (float)count * 2.0 * PI / counts_per_revolution;
    }

private:
    static void readEncoderISR() {
        if (instance != nullptr) {
            instance->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    float counts_per_revolution = 698; 
    volatile long count = 0;

private:
    static Encoder* instance;
};

Encoder* Encoder::instance = nullptr;

}  // namespace mtrn3100