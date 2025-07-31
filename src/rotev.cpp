#include "rotev.h"

#include "Arduino.h"

#define LEDR 23
#define LEDG 22
#define LEDB 21

#define IMU_MISO 16
#define IMU_CS 17
#define IMU_SCK 18
#define IMU_MOSI 19

Rotev::Rotev() : mpu(SPI, IMU_CS, Mpu6x00::GYRO_2000DPS, Mpu6x00::ACCEL_16G) {
  // Constructor initializes MPU6500 with default settings
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
}

void Rotev::update() { this->mpu.readSensor(); }

float Rotev::getYaw() { return this->mpu.getGyroZ(); }

void Rotev::ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (int)(r * 255.0f));
  analogWrite(LEDG, (int)(g * 255.0f));
  analogWrite(LEDB, (int)(b * 255.0f));
}