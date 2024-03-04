/*
eepromWrite.ino, part of StratoSoar MK2, for an autonomous glider.
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

// Simply upload this code to the main MCU (SAMD21G18A) to write data


// IMPORTANT: settings.h must be the same file as in the autopilot directory.
// Stupid Arduino doesn't allow for relative paths...

#include "settings.h"
#include <Wire.h>

#define WRITE_TIME 10 // How many ms before writing a byte to the EEPROM?

long address, start;

int counter;

int timeEstimate = WRITE_TIME * MAX_ADDRESS / 1000;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  pinMode(RUDDER_BJT, OUTPUT);
  pinMode(ELEVATOR_BJT, OUTPUT);
  pinMode(PARACHUTE_BJT, OUTPUT);
  pinMode(RUDDER_FET, OUTPUT);
  pinMode(ELEVATOR_FET, OUTPUT);
  pinMode(PARACHUTE_FET, OUTPUT);
  pinMode(WAKEUP_PIN, OUTPUT);
  pinMode(FALSE_WAKEUP_PIN, INPUT);
  pinMode(BAT_VOLTAGE_PIN, INPUT);
  pinMode(WRITE_PIN, OUTPUT);
  digitalWrite(LED, LOW);
  digitalWrite(ERR_LED, LOW);
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(PARACHUTE_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
  digitalWrite(ELEVATOR_FET, LOW);
  digitalWrite(PARACHUTE_FET, LOW);
  digitalWrite(WAKEUP_PIN, LOW);
  digitalWrite(WRITE_PIN, LOW);

  SerialUSB.begin(9600);
  SerialUSB.print("Time estimate in seconds: ");
  SerialUSB.println(timeEstimate);
  SerialUSB.println("Beginning writing in ten seconds...");
  Wire.begin();
  delay(10000);
  start = millis();
}

void loop() {
  if (address < MAX_ADDRESS) {
    if (counter <= 255) {
      writeToEEPROM(EEPROM_I2C_ADDRESS, address, counter);
      counter++;
      address++;
    } else {
      counter = 0;
    }
  } else {
    SerialUSB.print("Time took: ");
    SerialUSB.println(millis() - start);
    while(1)
      ;
  }
}

void writeToEEPROM(byte EEPROMAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(EEPROMAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
  delay(5);
}