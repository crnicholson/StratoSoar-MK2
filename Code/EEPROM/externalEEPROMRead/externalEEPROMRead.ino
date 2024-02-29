/*
eepromRead.ino, part of StratoSoar MK2, for an autonomous glider.
Copyright (C) 2024 Charles Nicholson

This program is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
