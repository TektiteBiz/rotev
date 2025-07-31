#include "drv8873.h"

/**
 * @brief Constructor for the DRV8873 driver.
 * @param cs_pin The Arduino pin connected to the DRV8873's nSCS (Chip Select)
 * pin.
 */
DRV8873_SPI::DRV8873_SPI(uint8_t cs_pin) {
  _cs_pin = cs_pin;
  // DRV8873 supports SPI clock up to 10 MHz.
  // Mode 1 (CPOL=0, CPHA=1) is typical for TI drivers.
  _spi_settings = SPISettings(4000000, MSBFIRST, SPI_MODE1);
}

/**
 * @brief Initializes the driver and SPI communication.
 */
void DRV8873_SPI::begin() {
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);  // Deselect the chip
  SPI.begin();
}

/**
 * @brief Performs a 16-bit SPI transaction.
 * @param command The 16-bit command word to send.
 * @return The 16-bit response word from the device.
 */
uint16_t DRV8873_SPI::spi_transfer(uint16_t command) {
  uint16_t response = 0;
  SPI.beginTransaction(_spi_settings);
  digitalWrite(_cs_pin, LOW);  // Select the chip
  response = SPI.transfer16(command);
  digitalWrite(_cs_pin, HIGH);  // Deselect the chip
  SPI.endTransaction();
  return response;
}

/**
 * @brief Writes an 8-bit value to a specified register.
 * @param address The 5-bit address of the register to write to.
 * @param data The 8-bit data to write into the register.
 * @return The 16-bit response word from the device.
 */
uint16_t DRV8873_SPI::writeRegister(uint8_t address, uint8_t data) {
  // Construct the 16-bit command for a write operation
  // Bit 14 = 0 for Write
  uint16_t command = 0;
  command |= (uint16_t)(address & 0x1F)
             << 9;  // Mask address to 5 bits and shift
  command |= data;  // Data goes in the lower 8 bits

  return spi_transfer(command);
}

/**
 * @brief Reads an 8-bit value from a specified register.
 * @param address The 5-bit address of the register to read from.
 * @return The 8-bit data byte from the register.
 */
uint8_t DRV8873_SPI::readRegister(uint8_t address) {
  // Construct the 16-bit command for a read operation
  // Bit 14 = 1 for Read
  uint16_t command = 0;
  command |= (1 << 14);  // Set the R/W bit to 1 for Read
  command |= (uint16_t)(address & 0x1F)
             << 9;  // Mask address to 5 bits and shift

  uint16_t response = spi_transfer(command);

  // The lower 8 bits of the response contain the register data
  return response & 0xFF;
}

/**
 * @brief Reads the FAULT status register (0x00).
 * @return An 8-bit byte containing all the fault flags.
 */
uint8_t DRV8873_SPI::getFaultStatus() {
  return readRegister(DRV8873_REG_FAULT);
}

/**
 * @brief Clears all latched fault flags by setting the CLR_FLT bit.
 */
void DRV8873_SPI::clearAllFaults() {
  // To clear faults, we must write a '1' to the CLR_FLT bit in the IC3 Control
  // Register. We should read the register first to preserve other settings.
  uint8_t reg_val = readRegister(DRV8873_REG_IC3_CTRL);
  reg_val |= DRV8873_IC3_CLR_FLT;  // Set the clear fault bit
  writeRegister(DRV8873_REG_IC3_CTRL, reg_val);
}

/**
 * @brief Checks the global FAULT bit.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the global fault bit is set.
 */
bool DRV8873_SPI::isGlobalFault(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_GLOBAL);
}

/**
 * @brief Checks for Over Temperature Warning.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the OTW bit is set.
 */
bool DRV8873_SPI::isOverTempWarning(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_OTW);
}

/**
 * @brief Checks for Under Voltage Lockout.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the UVLO bit is set.
 */
bool DRV8873_SPI::isUnderVoltage(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_UVLO);
}

/**
 * @brief Checks for Charge Pump Undervoltage.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the CPUV bit is set.
 */
bool DRV8873_SPI::isChargePumpFault(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_CPUV);
}

/**
 * @brief Checks for Over Current Protection.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the OCP bit is set.
 */
bool DRV8873_SPI::isOverCurrent(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_OCP);
}

/**
 * @brief Checks for Thermal Shutdown.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the TSD bit is set.
 */
bool DRV8873_SPI::isThermalShutdown(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_TSD);
}

/**
 * @brief Checks for Open-Load Detection.
 * @param fault_data The 8-bit byte read from the FAULT register.
 * @return True if the OLD bit is set.
 */
bool DRV8873_SPI::isOpenLoad(uint8_t fault_data) {
  return (fault_data & DRV8873_FAULT_OLD);
}