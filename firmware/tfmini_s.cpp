#include "tfmini_s.h"

bool TFMiniS::sendCommand(uint8_t *cmd, uint8_t len, const char *cmdName, uint16_t delayTime)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd, len);
  uint8_t error = Wire.endTransmission();

  delay(delayTime);
  return error == 0;
}

void TFMiniS::begin()
{
  Serial.print("\nInitializing TFMini-S at address 0x");
  Serial.println(addr, HEX);

  uint8_t resetCmd[] = {0x5A, 0x04, 0x02, 0x60};
  if (!sendCommand(resetCmd, 4, "reset", 100))
  {
    Serial.println("Reset failed");
    return;
  }

  uint8_t i2cModeCmd[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
  if (!sendCommand(i2cModeCmd, 5, "set I2C mode", 100))
  {
    Serial.println("Set I2C mode failed");
    return;
  }

  uint8_t formatCmd[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
  if (!sendCommand(formatCmd, 5, "set format", 100))
  {
    Serial.println("Set format failed");
    return;
  }

  uint8_t saveCmd[] = {0x5A, 0x04, 0x11, 0x6F};
  if (!sendCommand(saveCmd, 4, "save settings", 2000))
  {
    Serial.println("Save settings failed");
    return;
  }

  Serial.print("Initialization complete for sensor at 0x");
  Serial.println(addr, HEX);
}

bool TFMiniS::readDistance(uint16_t &distance)
{
  // TODO what if setup never succeeded?

  uint8_t getDataCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
  if (!sendCommand(getDataCmd, 5, "get data", 1))
  {
    return false;
  }

  if (Wire.requestFrom(addr, (uint8_t)9) != 9)
  {
    Serial.print("Failed to read from sensor 0x");
    Serial.println(addr, HEX);
    return false;
  }

  uint8_t buffer[9];
  for (int i = 0; i < 9; i++)
  {
    buffer[i] = Wire.read();
  }

  if (buffer[0] != 0x59 || buffer[1] != 0x59)
  {
    Serial.print("Invalid frame header from sensor 0x");
    Serial.println(addr, HEX);
    return false;
  }

  distance = buffer[2] | (buffer[3] << 8);

  return true;
}