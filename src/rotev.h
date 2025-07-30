#ifndef Rotev_h
#define Rotev_h

#include "Arduino.h"

class Rotev {
 public:
  void begin();
  void ledWrite(float r, float g, float b);
};

#endif