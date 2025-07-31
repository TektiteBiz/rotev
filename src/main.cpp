#include "rotev.h"

Rotev rotev;

void setup() {
  rotev.begin();
  rotev.motorEnable(true);
  rotev.motorWrite1(0.1f);
}

void loop() {
  rotev.update();

  Serial.println("gyro:" + String(rotev.getYaw()));
  rotev.ledWrite(0.0f, 0.1f, 0.0f);
  delay(25);
}
