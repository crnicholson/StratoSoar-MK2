// For use with AT24Cx EEPROM
// Connect SDA and SCL to EEPROM. Voltage does not matter. Tested with 3.3v and 5v.
// Charles Nicholson, 2023
// Use this code to read EEPROM data in conjuction with the StratoSoar autopilot
#include <Wire.h>
#define EEPROM_I2C_address 0x50
#define maxAddress 5000

int address = 0;
bool runEEPROM = true;

void setup() {                                                                     
  Serial.begin(9600);
  Serial.println("Beginning reading...");
  delay(250);
  Serial.println("Yaw, pitch, temperature (C), pressure (Pa)");
  Wire.begin();
}

void loop() {
  if (runEEPROM) { 
    Serial.print("Yaw: ");
    Serial.print(int(readFromEEPROM(EEPROM_I2C_address, address)) * 2);
    delay(10);
    address++;
    Serial.print(", Pitch: ");
    Serial.print(readFromEEPROM(EEPROM_I2C_address, address));
    delay(10);
    address++;
    Serial.print(", Temp: ");
    Serial.print(readFromEEPROM(EEPROM_I2C_address, address));
    delay(10);
    address++;
    Serial.print(", Pressure: ");
    Serial.println(int(readFromEEPROM(EEPROM_I2C_address, address)) * 500);
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
  if(Wire.available()) return Wire.read();
  return 0;
}