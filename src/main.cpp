#include "rotev.h"

// Motor 1 is left, motor 2 is right

Rotev rotev;
float prevAngle1 = 0.0f;
float prevAngle2 = 0.0f;
float pos1 = 0.0f;
float pos2 = 0.0f;
float heading = 0.0f;

const float CM_PER_RAD =
    (2.375f * 2.54f) / 2.0f;  // Wheel radius in cm, given 2.375" dia wheels
const float GYRO_OFFSET = 0.0202f;

void setup() {
  rotev.begin();
  rotev.motorEnable(true);
  rotev.motorWrite1(0.0f);
  rotev.motorWrite2(0.0f);
  rotev.servoWrite(25.0f);  // 25: closed, 180: open
  for (int i = 0; i < 10; i++) {
    prevAngle1 = rotev.enc1Angle();
    prevAngle2 = rotev.enc2Angle();
    delay(50);
  }

  rp2040.fifo.push(0);  // Send ready signal
}

#define MOTOR1_MULT -1.0f
#define MOTOR2_MULT 1.0f

void pushIref(float iref, bool motor1) {
  if (motor1) {
    iref *= MOTOR1_MULT;
  } else {
    iref *= MOTOR2_MULT;
  }
  int32_t fixed = (int32_t)(iref * 10000.0f);
  uint32_t val = ((uint32_t)fixed << 1);
  if (motor1) {
    val |= 0x1;  // Set bit 0 for motor 1
  } else {
    val &= ~0x1;  // Clear bit 0 for motor 2
  }
  rp2040.fifo.push(val);
}

float wrapDelta(float delta) {
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  return delta;
}

uint32_t prevTimeMicrosCore1 = 0;
float volt = 0.0f;
void loop() {
  if (rp2040.fifo.available()) {
    volt = rp2040.fifo.pop() / 1000.0f;  // Convert from mV to V
  }
  float dT = (float)(micros() - prevTimeMicrosCore1) /
             1000000.0f;  // Convert to seconds
  prevTimeMicrosCore1 = micros();

  float angle1 = rotev.enc1Angle();
  float angle2 = rotev.enc2Angle();
  float delta1 = wrapDelta(angle1 - prevAngle1) * CM_PER_RAD * MOTOR1_MULT;
  float delta2 = wrapDelta(angle2 - prevAngle2) * CM_PER_RAD * MOTOR2_MULT;
  float vel1 = delta1 / dT;
  float vel2 = delta2 / dT;
  heading += (rotev.readYaw() - GYRO_OFFSET) * dT;  // Heading in radians

  prevAngle1 = angle1;
  prevAngle2 = angle2;
  pos1 += delta1;
  pos2 += delta2;

  Serial.print("heading:" + String(heading));
  Serial.print(",volt:" + String(volt));
  Serial.print(",vel1:" + String(vel1));
  Serial.print(",vel2:" + String(vel2));
  Serial.print(",pos1:" + String(pos1));
  Serial.print(",pos2:" + String(pos2));
  Serial.println();
  if (rotev.stopButtonPressed()) {
    rotev.ledWrite(0.1f, 0.0f, 0.0f);
    pushIref(0.0f, true);
    pushIref(0.0f, false);
  } else if (rotev.goButtonPressed()) {
    rotev.ledWrite(0.0f, 0.1f, 0.0f);
    pushIref(0.7f, false);
    pushIref(0.7f, true);
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);
  }

  delay(50);
}

/*
25GA-370 (https://www.amazon.com/dp/B07ZKLZYRY)
- Resistance: 3.81 ohms
- Inductance: 158.5uH

Pololu 25D motor:
- Resistance: 2.26 ohms
- Inductance: 87.3uH
*/

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

float iref1 = 0.0f;
float iref2 = 0.0f;
void loop1() {
  float vbus = rotev.getVoltage();
  float dt =
      (float)(micros() - prevTimeMicros) / 1000000.0f;  // Convert to seconds
  piUpdate(dt, true, iref1, vbus);
  piUpdate(dt, false, iref2, vbus);
  prevTimeMicros = micros();
  delayMicroseconds(100);  // Run at 10khz max

  // Send over current data every 50ms
  if (millis() - lastWrite > 50) {
    lastWrite = millis();
    rp2040.fifo.push_nb((uint32_t)(rotev.getVoltage() * 1000.0f));
  }

  // Check for iref
  while (rp2040.fifo.available() > 0) {
    uint32_t val = rp2040.fifo.pop();
    bool motor1 = (val & 0x1) != 0;
    int32_t fixed = ((int32_t)val) >> 1;  // Remove motor bit
    float iref = fixed / 10000.0f;
    if (motor1) {
      iref1 = iref;
    } else {
      iref2 = iref;
    }
  }
}
