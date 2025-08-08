#include <SPI.h>

/**
 * @brief MT6701 Arduino Driver Class
 * * This class provides a simple and flexible interface for reading angle data
 * from an MT6701 magnetic encoder using the Arduino SPI library.
 * It is designed to be compatible with any SPI peripheral, including SPI1 on
 * RP2040-based boards.
 */
class MT6701 {
 public:
  /**
   * @brief Construct a new MT6701Driver object
   * * @param spi The SPIClass object to use (e.g., &SPI, &SPI1).
   * @param csPin The pin number connected to the MT6701's Chip Select (CS) pin.
   */
  MT6701(SPIClass* spi, int csPin) : _spi(spi), _csPin(csPin) {}

  /**
   * @brief Initializes the SPI communication and the CS pin.
   * * This function should be called once in the Arduino setup() function.
   * It sets the CS pin to OUTPUT and configures the SPI peripheral for
   * the MT6701's specific requirements (Mode 2, MSB first).
   */
  void begin() {
    // Set the Chip Select pin as an output and set it high initially
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // Begin the SPI transaction with the specified settings for MT6701.
    // The MT6701 datasheet specifies SPI Mode 2 (CPOL=0, CPHA=1).
    // The Arduino SPI library uses SPISettings(speed, bitOrder, dataMode).
    // The MT6701's clock speed can be up to 10MHz.
    _spi->begin();
    _spiSettings = SPISettings(8000000, MSBFIRST, SPI_MODE2);
  }

  /**
   * @brief Reads the raw 14-bit angle from the MT6701 sensor.
   * * This function performs the SPI transaction to get the 16-bit word from
   * the encoder and then masks the data to extract the 14-bit angle value.
   * * The MT6701's 16-bit data frame is structured as:
   * [Mag-H] [Mag-L] [14-bit Angle (D13...D0)]
   * D15     D14     D13...D0
   * * We need to mask out the top two bits (Mag-H and Mag-L). The mask 0x3FFF
   * achieves this perfectly.
   * * @return The raw 14-bit angle value (0-16383).
   */
  uint16_t readAngleRaw() {
    // Start the SPI transaction
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);

    // The MT6701 ignores MOSI during a read, so we send a dummy value.
    // The transfer16 function handles both transmit and receive.
    // It returns the 16-bit received word.
    uint16_t received_word = _spi->transfer16(0x0000);

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    // Mask top two bits and shift right 1 to get 14-bit angle
    return ((received_word >> 1) & 0x3FFF);
  }

  /**
   * @brief Reads the angle and converts it to degrees.
   * * @return The angle in degrees (0.0 to 360.0).
   */
  float readAngleDegrees() {
    uint16_t angle_raw = readAngleRaw();
    // The 14-bit value ranges from 0 to 16383.
    // 16384 is 2^14.
    return ((float)angle_raw / 16384.0f) * 360.0f;
  }

  /**
   * @brief Reads the angle and converts it to radians.
   * * @return The angle in radians (0.0 to 2*PI).
   */
  float readAngleRadians() {
    uint16_t angle_raw = readAngleRaw();
    return ((float)angle_raw / 16384.0f) * 2.0f * PI;
  }

 private:
  SPIClass* _spi;
  int _csPin;
  SPISettings _spiSettings;
};