#ifndef DRV8873_SPI_H
#define DRV8873_SPI_H

#include <Arduino.h>
#include <SPI.h>

// --- Register Map (Verified from Datasheet Image) ---
#define DRV8873_REG_FAULT 0x00     // FAULT Status (R)
#define DRV8873_REG_DIAG 0x01      // DIAG Status (R)
#define DRV8873_REG_IC1_CTRL 0x02  // IC1 Control (R/W)
#define DRV8873_REG_IC2_CTRL 0x03  // IC2 Control (R/W)
#define DRV8873_REG_IC3_CTRL 0x04  // IC3 Control (R/W)
#define DRV8873_REG_IC4_CTRL 0x05  // IC4 Control (R/W)

// --- Bitmasks for IC1_CTRL Register (0x02) ---
#define DRV8873_IC1_SPI_IN \
  (1 << 5)  // 1 = Outputs follow SPI registers, 0 = Outputs follow INx pins
#define DRV8873_IC1_MODE_PH_EN 0b00
#define DRV8873_IC1_MODE_PWM 0b01
#define DRV8873_IC1_MODE_IND 0b10
#define DRV8873_IC1_MODE_MASK 0b11

// --- Bitmasks for IC3_CTRL Register (0x04) ---
#define DRV8873_IC3_CLR_FLT (1 << 7)  // 1 = Clear fault status bits

// --- Bitmasks for FAULT Register (0x00) Data Byte ---
#define DRV8873_FAULT_GLOBAL (1 << 6)  // Global FAULT status register
#define DRV8873_FAULT_OTW (1 << 5)     // Overtemperature warning
#define DRV8873_FAULT_UVLO (1 << 4)    // UVLO fault condition
#define DRV8873_FAULT_CPUV (1 << 3)    // Charge-pump undervoltage fault
#define DRV8873_FAULT_OCP (1 << 2)     // Overcurrent condition
#define DRV8873_FAULT_TSD (1 << 1)     // Overtemperature shutdown
#define DRV8873_FAULT_OLD (1 << 0)     // Open-load detection

// --- Bitmasks for IC4_CTRL Register (0x05) ---
#define DRV8873_IC4_EN_OLP (1 << 6)   // Enable Open-Load Passive Diagnostic
#define DRV8873_IC4_OLP_DLY (1 << 5)  // Open-Load Passive Diagnostic Delay
#define DRV8873_IC4_EN_OLA (1 << 4)   // Enable Open-Load Active Diagnostic
#define DRV8873_IC4_ITRIP_LVL_MASK 0b1100     // Mask for ITRIP_LVL bits
#define DRV8873_IC4_ITRIP_LVL_00 (0b00 << 2)  // ITRIP_LVL: 00b
#define DRV8873_IC4_ITRIP_LVL_01 (0b01 << 2)  // ITRIP_LVL: 01b
#define DRV8873_IC4_ITRIP_LVL_10 (0b10 << 2)  // ITRIP_LVL: 10b
#define DRV8873_IC4_ITRIP_LVL_11 (0b11 << 2)  // ITRIP_LVL: 11b
#define DRV8873_IC4_DIS_ITRIP_MASK 0b11       // Mask for DIS_ITRIP bits
#define DRV8873_IC4_DIS_ITRIP_NONE 0b00  // Disable current regulation: None
#define DRV8873_IC4_DIS_ITRIP_OUT1 \
  0b01  // Disable current regulation: Only OUT1
#define DRV8873_IC4_DIS_ITRIP_OUT2 \
  0b10  // Disable current regulation: Only OUT2
#define DRV8873_IC4_DIS_ITRIP_BOTH \
  0b11  // Disable current regulation: Both OUT1 and OUT2

class DRV8873_SPI {
 public:
  // Constructor: Takes the SPI Chip Select (CS) pin
  DRV8873_SPI(uint8_t cs_pin);

  // Initialization: Starts SPI communication
  void begin();

  // Low-level register access
  uint16_t writeRegister(uint8_t address, uint8_t data);
  uint8_t readRegister(uint8_t address);

  // High-level functions
  uint8_t getFaultStatus();
  void clearAllFaults();

  // Status checking helper functions (take 8-bit fault data as input)
  bool isGlobalFault(uint8_t fault_data);
  bool isOverTempWarning(uint8_t fault_data);
  bool isUnderVoltage(uint8_t fault_data);
  bool isChargePumpFault(uint8_t fault_data);
  bool isOverCurrent(uint8_t fault_data);
  bool isThermalShutdown(uint8_t fault_data);
  bool isOpenLoad(uint8_t fault_data);

 private:
  uint8_t _cs_pin;
  SPISettings _spi_settings;

  // Core SPI transaction function
  uint16_t spi_transfer(uint16_t command);
};

#endif  // DRV8873_SPI_H