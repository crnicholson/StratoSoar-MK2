// For use with AT24CX EEPROM
// Connect SDA and SCL to EEPROM. Voltage does not matter. Tested with 3.3v and 5v.
// Charles Nicholson, 2023
// Use this code to clear the EEPROM to use in conjunction with the StratoSoar autopilot
#include <Wire.h>
#define EEPROM_I2C_address 0x50
#define maxAddress 5000

int address = 0;
bool runCode = true;

void setup() {                                                                     
  Serial.begin(9600);
  Serial.println("Clearing the EEPROM...");
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
