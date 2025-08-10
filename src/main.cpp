#include "rotev.h"

// Motor 1 is left, motor 2 is right

Rotev rotev;

const float CM_PER_RAD =
    (2.375f * 2.54f) / 2.0f;  // Wheel radius in cm, given 2.375" dia wheels
const float GYRO_OFFSET = 0.0202f;

void setup() {
  rotev.begin();
  rotev.servoWrite(180.0f);  // 25: closed, 180: open

  rp2040.fifo.push(0);  // Send ready signal
}

// Determined by just doing pushIref(0.8f, true) and looking at vel1/vel2
#define MOTOR1_MULT 1.0f
#define MOTOR2_MULT -1.0f
#define ENC1_MULT -1.0f
#define ENC2_MULT 1.0f

void pushVref(float iref, bool linvel) {
  int32_t fixed = (int32_t)(iref * 10000.0f);
  uint32_t val = ((uint32_t)fixed << 1);
  if (linvel) {
    val |= 0x1;  // Set bit 0 for motor 1
  } else {
    val &= ~0x1;  // Clear bit 0 for motor 2
  }
  rp2040.fifo.push(val);
}

uint32_t prevTimeMicrosCore1 = 0;
uint32_t lastPrintCore1 = 0;
float volt = 0.0f;

volatile float posX = 0.0f;
volatile float posY = 0.0f;
volatile float yawRate = 0.0f;
volatile float heading = 0.0f;

float kPx = 3.0f;
float kPh = 50.0f;
float kPh_d = 2.0f;  // Derivative gain for heading control

#define TARG_COUNT 18
float targX[TARG_COUNT] =
    {
        100.0f, 100.0f, 0.0f,   0.0f,   100.0f, 100.0f,
        0.0f,   0.0f,   100.0f, 100.0f, 0.0f,   0.0f,
        0.0f,   100.0f, 100.0f, -50.0f, -50.0f, 4.5f};  // Target position in cm
float targY[TARG_COUNT] = {0.0f,   100.0f, 100.0f, 0.0f,   0.0f, 100.0f, 100.0f,
                           0.0f,   0.0f,   100.0f, 100.0f, 0.0f, 0.0f,   0.0f,
                           100.0f, 100.0f, 0.0f,   -1.25f};  // Target
                                                             // position
                                                             // in cm
int currTarg = 0;

#define MAX_VEL 150.0f
#define MAX_ANG_VEL 30.0f

bool going = false;

bool goPressed = false;

extern float vel1;
extern float vel2;
float targVel1 = 0.0f;  // Target velocity for motor 1
float targVel2 = 0.0f;  // Target velocity for motor 2
void loop() {
  if (rp2040.fifo.available()) {
    volt = rp2040.fifo.pop() / 1000.0f;  // Convert from mV to V
  }

  // Calculate heading & position err
  float xErr = targX[currTarg] - posX;
  float yErr = targY[currTarg] - posY;
  float posErr = sqrtf(xErr * xErr + yErr * yErr);
  if (currTarg < TARG_COUNT - 1 && posErr < 16.0f) {
    currTarg++;
  }
  // Calculate heading to target
  float headingErr = heading - atan2f(yErr, xErr);
  while (headingErr < -M_PI) {
    headingErr += 2 * M_PI;
  }
  while (headingErr > M_PI) {
    headingErr -= 2 * M_PI;
  }

  // Update position controllers
  float v = kPx * posErr;
  if (v > MAX_VEL) {
    v = MAX_VEL;
  } else if (v < -MAX_VEL) {
    v = -MAX_VEL;
  }
  float w = kPh * headingErr + kPh_d * yawRate;  // Heading control
  if (w > MAX_ANG_VEL) {  // Max 0.25m/s diff between wheel speed
    w = MAX_ANG_VEL;
  } else if (w < -MAX_ANG_VEL) {
    w = -MAX_ANG_VEL;
  }
  if (going) {
    targVel1 = v;
    targVel2 = w;
    pushVref(v, true);
    pushVref(w, false);
  } else {
    pushVref(0.0f, true);
    pushVref(0.0f, false);
  }

  if (rotev.stopButtonPressed()) {
    rotev.ledWrite(0.1f, 0.0f, 0.0f);
    going = false;
    rotev.motorEnable(false);
    rotev.servoWrite(180.0f);
  } else if (rotev.goButtonPressed()) {
    rotev.ledWrite(0.0f, 0.1f, 0.0f);
    goPressed = true;
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);
  }
  if (!rotev.goButtonPressed() && goPressed) {
    goPressed = false;
    going = true;
    posX = 0.0f;
    posY = 0.0f;
    heading = 0.0f;
    targVel1 = 0.0f;
    targVel2 = 0.0f;
    currTarg = 0;
    rotev.motorEnable(true);
    rotev.servoWrite(25.0f);
  }

  if (millis() - lastPrintCore1 > 50) {
    lastPrintCore1 = millis();
    Serial.print("heading:" + String(heading));
    Serial.print(",volt:" + String(volt));
    Serial.print(",linvel:" + String((vel1 + vel2) / 2.0f * CM_PER_RAD));
    Serial.print(",angvel:" + String((vel2 - vel1) / 2.0f * CM_PER_RAD));
    Serial.print(",posX:" + String(posX));
    Serial.print(",posY:" + String(posY));
    Serial.print(",targlinvel:" + String(targVel1));
    Serial.print(",targangvel:" + String(targVel2));
    Serial.println();
  }
  delayMicroseconds(200);  // Run at 5khz max
}

/*
25GA-370 (https://www.amazon.com/dp/B07ZKLZYRY)
- Resistance: 3.81 ohms
- Inductance: 158.5uH

Pololu 25D motor:
- Resistance: 2.26 ohms
- Inductance: 87.3uH
*/

#define BANDWIDTH 160 * 2 * M_PI  // Bandwidth in Hz * 2pi
#define INDUCTANCE 158.5e-6       // Henries
#define RESISTANCE 3.81           // Ohms
#define MAX_DUTY 0.95f            // Duty cycle out of 1
#define RPM_12V 620.0f            // Max RPM of the motor
unsigned long prevTimeMicros = 0;
unsigned long lastWrite = 0;

const float kP = BANDWIDTH * INDUCTANCE;
const float kI = (RESISTANCE / INDUCTANCE) * BANDWIDTH * INDUCTANCE;
const float kV = 12.0f / (RPM_12V * (2.0f * M_PI / 60.0f));  // Voltage to rad/s

float mI1 = 0.0f;  // Integral term
float mI2 = 0.0f;  // Integral term
float dirr1 = 0.0f;
float dirr2 = 0.0f;
void piUpdate(float dt, bool motor1, float iref, float vbus, float vel) {
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
    rotev.motorWrite1(dd * MOTOR1_MULT);
  } else {
    rotev.motorWrite2(dd * MOTOR2_MULT);
  }

  // Estimate current sign
  float voltageMotor = vel * kV;
  float voltageApplied = dd * vbus;
  /*if (!motor1 && millis() - lastWrite > 50) {
    lastWrite = millis();
    Serial.print("voltageApplied:" + String(voltageApplied, 2));
    Serial.print(",voltageMotor:" + String(voltageMotor, 2));
    Serial.print(",vel:" + String(vel, 2));
    Serial.print(",curr:" + String(curr, 2));
    Serial.print(",dt:" + String(dt, 5));
    Serial.println();
  }*/
  float dir = (voltageApplied - voltageMotor) >= 0 ? 1.0f : -1.0f;
  if (motor1) {
    dirr1 = dir;
  } else {
    dirr2 = dir;
  }
}

float wrapDelta(float delta) {
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  } else if (delta < -M_PI) {
    delta += 2 * M_PI;
  }
  return delta;
}

// Vel calculations
float prevAngle1 = 0.0f;
float prevAngle2 = 0.0f;
float vel1 = 0.0f;  // Note: in rad/s
float vel2 = 0.0f;

void setup1() {
  rp2040.fifo.pop();  // Wait for ready signal
  prevTimeMicros = micros();
  lastWrite = millis();
  for (int i = 0; i < 10; i++) {
    prevAngle1 = rotev.enc1Angle();
    prevAngle2 = rotev.enc2Angle();
    delay(50);
  }
}

#define VEL_KP 0.03f  // Velocity proportional gain

// Past this it slips
#define IREF_MAX 0.3f
#define IREF_MAX_DECEL 0.15f

#define IREF_MAX_LIN 0.25f  // Max lin accel, leaves at least 0.1A for turning
#define IREF_MAX_DECEL_LIN 0.1f  // Max lin decel, 0.05A for turning

#define IREF_FRICTION 0.15f  // Current to overcome friction, 0.15A

float clampMagnitude(float val, float max) {
  if (val > max) {
    return max;
  } else if (val < -max) {
    return -max;
  }
  return val;
}

float vreflin = 0.0f;
float vrefang = 0.0f;
void loop1() {
  float vbus = rotev.getVoltage();
  float dt =
      (float)(micros() - prevTimeMicros) / 1000000.0f;  // Convert to seconds
  prevTimeMicros = micros();

  // Update velocities
  float angle1 = rotev.enc1Angle();
  float angle2 = rotev.enc2Angle();
  float delta1 = wrapDelta(angle1 - prevAngle1) * ENC1_MULT;
  float delta2 = wrapDelta(angle2 - prevAngle2) * ENC2_MULT;
  float vel1_raw = delta1 / dt;
  float vel2_raw = delta2 / dt;
  vel1 = (vel1 * 0.5f) + (vel1_raw * 0.5f);  // Low-pass filter
  vel2 = (vel2 * 0.5f) + (vel2_raw * 0.5f);
  prevAngle1 = angle1;
  prevAngle2 = angle2;

  // Update position
  float newYawRate = rotev.readYaw() - GYRO_OFFSET;
  float newHeading = heading + newYawRate * dt;  // Heading in radians
  if (newHeading < -M_PI) {
    newHeading += 2 * M_PI;
  } else if (newHeading > M_PI) {
    newHeading -= 2 * M_PI;
  }

  float delPos =
      (delta1 + delta2) / 2.0f * CM_PER_RAD;  // Average distance traveled
  float newPosX = posX + delPos * cosf(newHeading);
  float newPosY = posY + delPos * sinf(newHeading);

  // Write to shared memory
  yawRate = yawRate * 0.6f + newYawRate * 0.4f;  // Low-pass filter
  heading = newHeading;
  posX = newPosX;
  posY = newPosY;

  // Update velocity PID
  float currLinVel = (vel1 + vel2) / 2.0f;  // Average linear velocity
  float currAngVel = (vel1 - vel2) / 2.0f;  // Average angular velocity
  float ireflin = (vreflin - currLinVel) * VEL_KP;
  float irefang = (vrefang - currAngVel) * VEL_KP;

  // Current limiting based on accel vs decel
  float irefMaxLin = IREF_MAX_LIN;
  if ((currLinVel > 0.0f && vreflin < currLinVel) ||
      (currLinVel < 0.0f && vreflin > currLinVel)) {
    irefMaxLin = IREF_MAX_DECEL_LIN;
  }

  // Limit current for lin vel
  ireflin = clampMagnitude(ireflin, irefMaxLin);

  // Limit current per motor, to achieve irefang current limiting
  float iref1 = ireflin + irefang;
  float iref2 = ireflin - irefang;
  if (vel1 > 0.0f && ireflin < IREF_FRICTION ||
      vel1 < 0.0f && ireflin > -IREF_FRICTION) {  // Trying to decel (wheel 1)
    iref1 = clampMagnitude(iref1, IREF_MAX_DECEL);
  } else {
    iref1 = clampMagnitude(iref1, IREF_MAX);
  }
  if (vel2 > 0.0f && ireflin < IREF_FRICTION ||
      vel2 < 0.0f && ireflin > -IREF_FRICTION) {  // Trying to decel (wheel 2)
    iref2 = clampMagnitude(iref2, IREF_MAX_DECEL);
  } else {
    iref2 = clampMagnitude(iref2, IREF_MAX);
  }

  piUpdate(dt, true, ireflin + irefang, vbus, vel1);
  piUpdate(dt, false, ireflin - irefang, vbus, vel2);

  // Send over voltage data every 50ms
  if (millis() - lastWrite > 50) {
    lastWrite = millis();
    rp2040.fifo.push_nb((uint32_t)(rotev.getVoltage() * 1000.0f));
  }

  // Check for vref
  while (rp2040.fifo.available() > 0) {
    uint32_t val = rp2040.fifo.pop();
    bool linvel = (val & 0x1) != 0;
    int32_t fixed = ((int32_t)val) >> 1;  // Remove motor bit
    float vref = fixed / 10000.0f;
    if (linvel) {
      vreflin = vref / CM_PER_RAD;
    } else {
      vrefang = vref / CM_PER_RAD;
    }
  }

  /*if (going) {
    float dd = sinf(millis() / 1000.0f * 10.0f);  // Example input signal
    rotev.motorWrite2(dd * MOTOR2_MULT);
    float curr = rotev.motorCurr2() * dirr2;
    float voltageMotor = vel2 * kV;
    float voltageApplied = dd * vbus;
    dirr2 = (voltageApplied - voltageMotor) >= 0 ? 1.0f : -1.0f;
    Serial.print("voltageApplied:" + String(voltageApplied, 2));
    Serial.print(",voltageMotor:" + String(voltageMotor, 2));
    Serial.print(",vel:" + String(vel2 * 0.1f, 2));
    Serial.print(",curr:" + String(curr * 60.0f, 2));
    Serial.print(",dt:" + String(dt, 5));
    Serial.println();
    delay(50);
  } else {
    rotev.motorWrite2(0.0f);
  }*/
}
