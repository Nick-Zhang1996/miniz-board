#include <Arduino.h>
#include <Wire.h>

class TFMiniS
{
private:
  uint8_t addr;

  bool sendCommand(const uint8_t *cmd, uint8_t len, const char *cmdName, uint16_t delayTime = 10);

public:
  TFMiniS(uint8_t address) : addr(address) {};

  void begin();
  bool readDistance(uint16_t &distance);

  void setI2CAddress(uint16_t addr);
};

namespace Commands
{ // from https://cdn.sparkfun.com/assets/8/a/f/a/c/16977-TFMini-S_-_Micro_LiDAR_Module-Product_Manual.pdf
  constexpr uint8_t RESET_CMD[] = {0x5A, 0x04, 0x02, 0x60};
  constexpr uint8_t I2C_MODE_CMD[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
  constexpr uint8_t OUTPUT_FORMAT_CM_CMD[] = {0x5A, 0x05, 0x05, 0x01, 0x65}; // set output format to 9-bytes (cm)
  constexpr uint8_t SAVE_SETTINGS_CMD[] = {0x5A, 0x04, 0x11, 0x6F};

  constexpr uint8_t GET_DATA_CMD[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
}