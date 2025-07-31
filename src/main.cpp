#include "rotev.h"

Rotev rotev;
void currLoop();

void setup() {
  rotev.begin();
  rotev.motorEnable(true);
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);

  rp2040.fifo.push(0);  // Send ready signal
}

void loop() {
  uint32_t curr = rp2040.fifo.pop();

  rotev.update();

  Serial.print("gyro:" + String(rotev.getYaw()));
  Serial.print(",curr:" + String(curr / 1000.0f));  // Convert to Amps
  Serial.println();
  if (rotev.stopButtonPressed()) {
    rotev.ledWrite(0.1f, 0.0f, 0.0f);
  } else if (rotev.goButtonPressed()) {
    rotev.ledWrite(0.0f, 0.1f, 0.0f);
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);
  }
}

#define BANDWIDTH 120 * 2 * M_PI  // Bandwidth in Hz * 2pi
#define INDUCTANCE 158.5e-6       // Henries
#define RESISTANCE 3.81           // Ohms
#define MAX_DUTY 0.95f            // Duty cycle out of 1
unsigned long prevTimeMicros = 0;
unsigned long lastWrite = 0;

const float kP = BANDWIDTH * INDUCTANCE;
const float kI = (RESISTANCE / INDUCTANCE) * BANDWIDTH * INDUCTANCE;

float mI1 = 0.0f;  // Integral term
float mI2 = 0.0f;  // Integral term
float dirr1 = 0.0f;
float dirr2 = 0.0f;
void piUpdate(float dt, bool motor1, float iref, float vbus) {
  float curr =
      motor1 ? (rotev.motorCurr1() * dirr1) : (rotev.motorCurr2() * dirr2);
  float error = iref - curr;

  float vd = kP * error + (motor1 ? mI1 : mI2);

  // Convert from voltage to duty cycle
  float dd = vd / vbus;

  // Integrator anti-windup
  float dd_scale = MAX_DUTY / fabsf(dd);
  if (dd_scale < 1.0f) {
    dd *= dd_scale;
    if (motor1) {
      mI1 *= 0.99f;
    } else {
      mI2 *= 0.99f;
    }
  } else {
    float incr = kI * error * dt;
    if (motor1) {
      mI1 += incr;
    } else {
      mI2 += incr;
    }
  }

  // Write PWM
  if (motor1) {
    rotev.motorWrite1(dd);
  } else {
    rotev.motorWrite2(dd);
  }

  // Update direction
  if (dd > 0) {
    if (motor1) {
      dirr1 = 1.0f;
    } else {
      dirr2 = 1.0f;
    }
  } else {
    if (motor1) {
      dirr1 = -1.0f;
    } else {
      dirr2 = -1.0f;
    }
  }
}

void setup1() {
  rp2040.fifo.pop();  // Wait for ready signal
  prevTimeMicros = micros();
  lastWrite = millis();
}

void loop1() {
  float vbus = rotev.getVoltage();
  float dt =
      (float)(micros() - prevTimeMicros) / 1000000.0f;  // Convert to seconds
  piUpdate(dt, false, 0.2f, vbus);
  prevTimeMicros = micros();
  delayMicroseconds(500);  // Run at 20khz max

  // Send over current data every 50ms
  if (millis() - lastWrite > 50) {
    lastWrite = millis();
    rp2040.fifo.push_nb((uint32_t)(rotev.motorCurr2() * 1000.0f));
  }
}
