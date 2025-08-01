#include "rotev.h"

#include "Arduino.h"

#define LEDR 23
#define LEDG 22
#define LEDB 21

#define IMU_MISO 16
#define IMU_CS 17
#define IMU_SCK 18
#define IMU_MOSI 19

#define DRV1_CS 7
#define DRV1_EN 6
#define DRV1_PH 5
#define DRV1_DISABLE 4
#define DRV1_CURR 26

#define DRV2_CS 3
#define DRV2_EN 2
#define DRV2_PH 1
#define DRV2_DISABLE 0
#define DRV2_CURR 27

#define ENC_SCK 10
#define ENC_MOSI 11
#define ENC_MISO 12
#define ENC1_CS 13
#define ENC2_CS 14

#define STOP 8
#define GO 9

#define SERVO 24

#define VBUS 28

Rotev::Rotev()
    : mpu(SPI, IMU_CS, Mpu6x00::GYRO_2000DPS, Mpu6x00::ACCEL_16G),
      driver1(DRV1_CS),
      driver2(DRV2_CS),
      enc1(&SPI1, ENC1_CS),
      enc2(&SPI1, ENC2_CS) {
  // Constructor initializes MPU6500 with default settings
}

int initMotor(DRV8873_SPI& drv, bool one) {
  // Configure for PH/EN mode
  uint8_t ic1_ctrl = drv.readRegister(DRV8873_REG_IC1_CTRL);
  // Set the MODE bits to PH/EN (00b)
  ic1_ctrl &= ~DRV8873_IC1_MODE_MASK;
  ic1_ctrl |= DRV8873_IC1_MODE_PH_EN;
  // Set SPI_IN to 0 to use INx pins
  ic1_ctrl &= ~DRV8873_IC1_SPI_IN;
  drv.writeRegister(DRV8873_REG_IC1_CTRL, ic1_ctrl);

  // Disable open-load protection
  /*uint8_t ic4_ctrl = drv.readRegister(DRV8873_REG_IC4_CTRL);
  ic4_ctrl &= ~(DRV8873_IC4_EN_OLP | DRV8873_IC4_EN_OLA);
  drv.writeRegister(DRV8873_REG_IC4_CTRL, ic4_ctrl);*/

  // Check faults
  drv.clearAllFaults();
  delay(1);
  uint8_t fault_status = drv.getFaultStatus();
  if (fault_status != 0) {
    uint8_t status2 = drv.readRegister(DRV8873_REG_DIAG);
    Serial.println("Motor " + String(one ? "1" : "2") +
                   " initialization failed with fault status: 0x" +
                   String(fault_status, HEX) + ", DIAG status: 0x" +
                   String(status2, HEX));
    return -1;
  }

  return 0;
}

void Rotev::begin() {
  // LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  this->ledWrite(0.1, 0.1, 0.1);

  // Serial
  Serial.begin(115200);

  // MPU
  SPI.setSCK(IMU_SCK);
  SPI.setMISO(IMU_MISO);
  SPI.setMOSI(IMU_MOSI);
  SPI.begin();
  if (!mpu.begin()) {
    while (1) {
      ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
      Serial.println("MPU initialization failed!");
      delay(1000);
    }
  }

  // Analog
  analogReadResolution(12);
  pinMode(DRV1_CURR, INPUT);
  pinMode(DRV2_CURR, INPUT);
  pinMode(VBUS, INPUT);
  pinMode(STOP, INPUT_PULLUP);
  pinMode(GO, INPUT_PULLUP);

  // Servo
  pinMode(SERVO, OUTPUT);
  this->servo.attach(SERVO);
  this->servo.write(90);  // Center the servo

  // Motor drivers
  pinMode(DRV1_CS, OUTPUT);
  pinMode(DRV1_EN, OUTPUT);
  pinMode(DRV1_PH, OUTPUT);
  pinMode(DRV1_DISABLE, OUTPUT);

  pinMode(DRV2_CS, OUTPUT);
  pinMode(DRV2_EN, OUTPUT);
  pinMode(DRV2_PH, OUTPUT);
  pinMode(DRV2_DISABLE, OUTPUT);

  analogWriteFreq(10000);     // 10000 Hz = 10 kHz
  analogWriteResolution(16);  // 16-bit resolution

  this->motorEnable(false);  // Disable motors initially
  this->motorWrite1(0.0f);
  this->motorWrite2(0.0f);

  driver1.begin();
  driver2.begin();

  int status = initMotor(driver1, true);
  while (status != 0) {
    ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
    delay(1000);
    status = initMotor(driver1, true);
  }
  status = initMotor(driver2, false);
  while (status != 0) {
    ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
    delay(1000);
    status = initMotor(driver2, false);
  }

  // Encoders
  SPI1.setSCK(ENC_SCK);
  SPI1.setMISO(ENC_MISO);
  SPI1.setMOSI(ENC_MOSI);
  SPI1.begin();
  enc1.begin();
  enc2.begin();
}

float Rotev::readYaw() {
  this->mpu.readSensor();
  return this->mpu.getGyroZ();
}

void Rotev::ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (uint16_t)(r * 65535.0f));
  analogWrite(LEDG, (uint16_t)(g * 65535.0f));
  analogWrite(LEDB, (uint16_t)(b * 65535.0f));
}

void Rotev::motorEnable(bool enable) {
  if (enable) {
    digitalWrite(DRV1_DISABLE, LOW);
    digitalWrite(DRV2_DISABLE, LOW);
  } else {
    digitalWrite(DRV1_DISABLE, HIGH);
    digitalWrite(DRV2_DISABLE, HIGH);
  }
}

void Rotev::motorWrite1(float speed) {
  digitalWrite(DRV1_PH, speed >= 0.0f ? HIGH : LOW);
  // analogWrite(DRV1_EN, (uint16_t)(fabsf(speed) * 65535.0f));
}

void Rotev::motorWrite2(float speed) {
  digitalWrite(DRV2_PH, speed >= 0.0f ? HIGH : LOW);
  analogWrite(DRV2_EN, (uint16_t)(fabsf(speed) * 65535.0f));
}

float Rotev::getVoltage() {
  // 18kohm R1 5.1kohm R2
  int raw = analogRead(VBUS);
  float voltage = ((float)raw * 3.3f / 4096.0f) * (18.0f + 5.1f) / 5.1f;
  return voltage;
}

float Rotev::motorCurr1() {
  int raw = analogRead(DRV1_CURR);
  float voltage = (float)raw * 3.3f / 4096.0f;
  return voltage / 680.0f * 1100.0f;  // 680-ohm resistor, 1100 gain
}

float Rotev::motorCurr2() {
  int raw = analogRead(DRV2_CURR);
  float voltage = (float)raw * 3.3f / 4096.0f;
  return voltage / 680.0f * 1100.0f;  // 680-ohm resistor, 1100 gain
}

bool Rotev::stopButtonPressed() { return digitalRead(STOP) == LOW; }
bool Rotev::goButtonPressed() { return digitalRead(GO) == LOW; }

float Rotev::enc1Angle() { return enc1.readAngleDegrees(); }

float Rotev::enc2Angle() { return enc2.readAngleDegrees(); }