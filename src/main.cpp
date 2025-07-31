#include "rotev.h"

Rotev rotev;

void setup() { rotev.begin(); }

void loop() {
  rotev.update();

  Serial.println("gyro:" + String(rotev.getYaw()));
  rotev.ledWrite(0.0f, 0.1f, 0.0f);
  delay(25);
}
