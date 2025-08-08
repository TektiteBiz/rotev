#ifndef Rotev_h
#define Rotev_h

#include <Servo.h>

#include "Arduino.h"
#include "drv8873.h"
#include "mpu6x00.h"
#include "mt6701.h"

class Rotev {
 public:
  Rotev();

  void begin();
  void ledWrite(float r, float g, float b);
  float readYaw();
  float readYawDegrees();
  void motorEnable(bool enable);
  void motorWrite1(float speed);
  void motorWrite2(float speed);
  float motorCurr1();
  float motorCurr2();
  float getVoltage();
  bool stopButtonPressed();
  bool goButtonPressed();
  float enc1Angle();
  float enc2Angle();
  float enc1AngleDegrees();
  float enc2AngleDegrees();
  void servoDetach();
  void servoWrite(float angle);

 private:
  Mpu6500 mpu;
  DRV8873_SPI driver1;
  DRV8873_SPI driver2;
  Servo servo;
  MT6701 enc1;
  MT6701 enc2;
};

#endif