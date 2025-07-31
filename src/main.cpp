#include "rotev.h"

Rotev rotev;

void setup() {
  rotev.begin();
  rotev.motorEnable(true);
  rotev.motorWrite1(0.1f);
  rotev.motorWrite2(0.1f);
}

void loop() {
  rotev.update();

  Serial.print("gyro:" + String(rotev.getYaw()));
  Serial.print(",voltage:" + String(rotev.getVoltage()));
  Serial.println();
  if (rotev.stopButtonPressed()) {
    rotev.ledWrite(0.1f, 0.0f, 0.0f);
  } else if (rotev.goButtonPressed()) {
    rotev.ledWrite(0.0f, 0.1f, 0.0f);
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);
  }
  delay(25);
}
