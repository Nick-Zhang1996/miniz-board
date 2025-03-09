#include "tfmini_s.h"

using namespace Commands;

bool TFMiniS::sendCommand(const uint8_t *cmd, uint8_t len, const char *cmdName, uint16_t delayTime)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd, len);
  uint8_t error = Wire.endTransmission();

  delay(delayTime);
  if (error != 0)
  {
    Serial.print("Error sending command to sensor: ");
    Serial.println(error);
  }
  return error == 0;
}

void TFMiniS::begin()
{
  // skip initialization, assume proper settings are already burned into sensor

  // Serial.print("\nInitializing TFMini-S at address 0x");
  // Serial.println(addr, HEX);

  // if (!sendCommand(RESET_CMD, 4, "reset", 100)) // should be 1000ms according to datasheet
  // {
  //   Serial.println("Reset failed");
  //   return;
  // }

  // if (!sendCommand(I2C_MODE_CMD, 5, "set I2C mode", 100))
  // {
  //   Serial.println("Set I2C mode failed");
  //   return;
  // }

  // if (!sendCommand(OUTPUT_FORMAT_CM_CMD, 5, "set format", 100))
  // {
  //   Serial.println("Set format failed");
  //   return;
  // }

  // if (!sendCommand(SAVE_SETTINGS_CMD, 4, "save settings", 2000))
  // {
  //   Serial.println("Save settings failed");
  //   return;
  // }

  // Serial.print("Initialization complete for sensor at 0x");
  // Serial.println(addr, HEX);
}

bool TFMiniS::readDistance(uint16_t &distance)
{
  // TODO what if setup never succeeded?

  if (!sendCommand(GET_DATA_CMD, 5, "get data", 1))
  {
    Serial.print("Failed to send get data command to sensor 0x");
    Serial.println(addr, HEX);
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

// UNTESTED, prefer using Benewake software to set settings over Serial (sensors come in serial mode from factory)
void TFMiniS::setI2CAddress(uint16_t addr)
{
  uint8_t set_addr_cmd[] = {0x5A, 0x05, 0x0B, addr, 106 + addr};

  if (!sendCommand(set_addr_cmd, 5, "set i2c address", 100))
  {
    Serial.println("Set I2C address failed");
    return;
  }
  
  if (!sendCommand(SAVE_SETTINGS_CMD, 4, "save settings", 2000))
  {
    Serial.println("Save settings failed");
    return;
  }

  Serial.print("Setting I2C address complete for sensor at 0x");
  Serial.println(addr, HEX);
}