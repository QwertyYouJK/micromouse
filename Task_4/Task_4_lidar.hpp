#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

namespace mtrn3100 {


class Lidar {
public:
  explicit Lidar(uint8_t xshutPin)
    : _xshut(xshutPin)
  {
    pinMode(_xshut, OUTPUT);
    digitalWrite(_xshut, LOW);  
  }

 
  void disable() {
    digitalWrite(_xshut, LOW);
  }

  bool begin(uint8_t address = 0x29) {
    digitalWrite(_xshut, HIGH);
    delay(50);
    Wire.begin();            
    sensor.init();
    sensor.configureDefault();
    sensor.setAddress(address);
    return true;
  }

  /** One‐shot range in millimetres */
  uint16_t readMillimetres() {
    return sensor.readRangeSingleMillimeters();
  }

  bool timeoutOccurred() {
    return sensor.timeoutOccurred();
  }

private:
  VL6180X sensor;
  uint8_t  _xshut;
};

} 
