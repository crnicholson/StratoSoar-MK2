// Part of StratoSoar MK2.
// externalEEPROMRead.ino.
// Charles Nicholson, 2023.
// Used to read data from the autopilot's EEPROM.
// https://github.com/crnicholson/StratoSoar-MK2/.

// Simply upload this code to the main MCU (SAMD21G18A) to obtain data.

#include <Wire.h>

#define EEPROM_I2C_address 0x50
#define maxAddress 5000

int address = 0;
bool runEEPROM = true;

void setup() {
  SerialUSB.begin(9600);
  SerialUSB.println("Beginning reading...");
  delay(250);
  SerialUSB.println("Yaw, pitch, temperature (C), pressure (Pa)");
  Wire.begin();
}

void loop() {
  if (runEEPROM) {
    SerialUSB.print("Yaw: ");
    SerialUSB.print(int(readFromEEPROM(EEPROM_I2C_address, address)) * 2);
    delay(10);
    address++;
    SerialUSB.print(", Pitch: ");
    SerialUSB.print(readFromEEPROM(EEPROM_I2C_address, address));
    delay(10);
    address++;
    SerialUSB.print(", Temp: ");
    SerialUSB.print(readFromEEPROM(EEPROM_I2C_address, address));
    delay(10);
    address++;
    SerialUSB.print(", Pressure: ");
    SerialUSB.println(int(readFromEEPROM(EEPROM_I2C_address, address)) * 500);
    delay(10);
    address++;
    if (address >= maxAddress) {
      runEEPROM = false;
    }
    delay(1000);
  }
}

byte readFromEEPROM(byte EEPROMAddress, byte dataAddress) {
  Wire.beginTransmission(EEPROMAddress);
  Wire.write(dataAddress);
  Wire.endTransmission();
  delay(5);
  Wire.requestFrom(EEPROMAddress, 1);
  if (Wire.available())
    return Wire.read();
  return 0;
}