#ifndef Rotev_h
#define Rotev_h

#include "Arduino.h"
#include "mpu6x00.h"

class Rotev {
 public:
  Rotev();

  void begin();
  void update();
  void ledWrite(float r, float g, float b);
  float getYaw();

 private:
  Mpu6500 mpu;
};

#endif