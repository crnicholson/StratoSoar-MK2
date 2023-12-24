// Part of StratoSoar MK2.
// externalEEPROMClear.ino.
// Charles Nicholson, 2023.
// Used to clear data from the autopilot's EEPROM.
// https://github.com/crnicholson/StratoSoar-MK2/.

// Simply upload this code to the main MCU (SAMD21G18A) to clear data registers.

#include <Wire.h>

#define EEPROM_I2C_address 0x50
#define maxAddress 5000

    int address = 0;
bool runCode = true;

void setup() {                                                                     
  SerialUSB.begin(9600);
  SerialUSB.println("Clearing the EEPROM...");
  Wire.begin();
  delay(5000);
}

void loop() {
  if (runCode) { 
    writeToEEPROM(EEPROM_I2C_address, address, 0);
    delay(10);
    address++;
    if (address >= maxAddress) {
      runCode = false;
      SerialUSB.println("Done.");
    }
  }
}

void writeToEEPROM(byte EEPROMAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(EEPROMAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
  delay(5);
}
