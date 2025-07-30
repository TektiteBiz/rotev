#include "rotev.h"

Rotev rotev;

void setup() { rotev.begin(); }

void loop() {
  Serial.println("Hello, Rotev!");
  rotev.ledWrite(0.0f, 0.1f, 0.0f);
  delay(1000);
}
