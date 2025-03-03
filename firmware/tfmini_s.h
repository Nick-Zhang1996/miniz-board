#include <Arduino.h>
#include <Wire.h>

class TFMiniS
{
private:
  uint8_t addr;

  bool sendCommand(uint8_t *cmd, uint8_t len, const char *cmdName, uint16_t delayTime = 10);

public:
  TFMiniS(uint8_t address) : addr(address) {};

  void begin();
  bool readDistance(uint16_t &distance);
};