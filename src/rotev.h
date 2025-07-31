#ifndef Rotev_h
#define Rotev_h

#include <Servo.h>

#include "Arduino.h"
#include "drv8873.h"
#include "mpu6x00.h"

class Rotev {
 public:
  Rotev();

  void begin();
  void update();
  void ledWrite(float r, float g, float b);
  float getYaw();
  void motorEnable(bool enable);
  void motorWrite1(float speed);
  void motorWrite2(float speed);
  float getVoltage();
  bool stopButtonPressed();
  bool goButtonPressed();

 private:
  Mpu6500 mpu;
  DRV8873_SPI driver1;
  DRV8873_SPI driver2;
  Servo servo;
};

#endif