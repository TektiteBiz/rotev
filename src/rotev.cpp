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
#define DRV1_PH 8
#define DRV1_DISABLE 4

#define DRV2_CS 3
#define DRV2_EN 2
#define DRV2_PH 1
#define DRV2_DISABLE 0

Rotev::Rotev()
    : mpu(SPI, IMU_CS, Mpu6x00::GYRO_2000DPS, Mpu6x00::ACCEL_16G),
      driver1(DRV1_CS),
      driver2(DRV2_CS) {
  // Constructor initializes MPU6500 with default settings
}

int initMotor(DRV8873_SPI& drv) {
  drv.clearAllFaults();
  delay(1);
  uint8_t fault_status = drv.getFaultStatus();
  if (fault_status != 0) {
    uint8_t status2 = drv.readRegister(DRV8873_REG_DIAG);
    Serial.println("Motor initialization failed with fault status: " +
                   String(fault_status, HEX) +
                   ", DIAG status: " + String(status2, HEX));
    return -1;
  }

  // Configure for PH/EN mode
  uint8_t ic1_ctrl = drv.readRegister(DRV8873_REG_IC1_CTRL);
  // Set the MODE bits to PH/EN (00b)
  ic1_ctrl &= ~DRV8873_IC1_MODE_MASK;
  ic1_ctrl |= DRV8873_IC1_MODE_PH_EN;
  // Set SPI_IN to 0 to use INx pins
  ic1_ctrl &= ~DRV8873_IC1_SPI_IN;
  drv.writeRegister(DRV8873_REG_IC1_CTRL, ic1_ctrl);

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
  SPI.setCS(IMU_CS);
  SPI.begin();
  if (!mpu.begin()) {
    while (1) {
      ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
      Serial.println("MPU initialization failed!");
      delay(1000);
    }
  }

  // Motor drivers
  pinMode(DRV1_CS, OUTPUT);
  pinMode(DRV1_EN, OUTPUT);
  pinMode(DRV1_PH, OUTPUT);
  pinMode(DRV1_DISABLE, OUTPUT);

  pinMode(DRV2_CS, OUTPUT);
  pinMode(DRV2_EN, OUTPUT);
  pinMode(DRV2_PH, OUTPUT);
  pinMode(DRV2_DISABLE, OUTPUT);

  this->motorEnable(false);  // Disable motors initially
  this->motorWrite1(0.0f);
  this->motorWrite2(0.0f);

  driver1.begin();
  driver2.begin();

  int status = initMotor(driver1);
  while (status != 0) {
    ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
    delay(1000);
    status = initMotor(driver1);
  }
  status = initMotor(driver2);
  while (status != 0) {
    ledWrite(0.25f, 0.0f, 0.0f);  // Red LED on error
    delay(1000);
    status = initMotor(driver2);
  }
}
void Rotev::update() { this->mpu.readSensor(); }

float Rotev::getYaw() { return this->mpu.getGyroZ(); }

void Rotev::ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (int)(r * 255.0f));
  analogWrite(LEDG, (int)(g * 255.0f));
  analogWrite(LEDB, (int)(b * 255.0f));
}

void Rotev::motorEnable(bool enable) {
  if (enable) {
    digitalWrite(DRV1_EN, LOW);
    digitalWrite(DRV2_EN, LOW);
  } else {
    digitalWrite(DRV1_EN, HIGH);
    digitalWrite(DRV2_EN, HIGH);
  }
}

void Rotev::motorWrite1(float speed) {
  digitalWrite(DRV1_PH, speed >= 0 ? HIGH : LOW);
  analogWrite(DRV1_CS, (uint8_t)(abs(speed) * 255.0f));
}

void Rotev::motorWrite2(float speed) {
  digitalWrite(DRV2_PH, speed >= 0 ? HIGH : LOW);
  analogWrite(DRV2_CS, (uint8_t)(abs(speed) * 255.0f));
}