#include "rotev.h"

#include "Arduino.h"

#define LEDR 23
#define LEDG 22
#define LEDB 21

void Rotev::begin() {
  // LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  this->ledWrite(0.1, 0.1, 0.1);

  // Serial
  Serial.begin(115200);
}

void Rotev::ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (int)(r * 255.0f));
  analogWrite(LEDG, (int)(g * 255.0f));
  analogWrite(LEDB, (int)(b * 255.0f));
}