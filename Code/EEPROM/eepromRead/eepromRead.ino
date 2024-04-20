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

// This code is a test to see if the Sparkfun library works better than my code.

// Simply upload this code to the main MCU (SAMD21G18A) to obtain data.

// OPTIONAL: use puTTY so you can download a .csv file output.

// IMPORTANT: copy and paste pin definitions from main settings.h if you've changed the pins and aren't using the PCB.
// Stupid Arduino doesn't allow for relative paths...

#define RUDDER_PIN 2        // Hardware pin 23. Servo signal pin that attaches to "input" of BJT.
#define ELEVATOR_PIN 3      // Hardware pin 14. Servo signal pin that attaches to "input" of BJT.
#define PARACHUTE_PIN 4     // Hardware pin 13. Servo signal pin that attaches to "input" of BJT.
#define RUDDER_FET 5        // Hardware pin 24. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define ELEVATOR_FET 6      // Hardware pin 29. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define PARACHUTE_FET 7     // Hardware pin 30. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define RUDDER_BJT 8        // Hardware pin 11. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define PARACHUTE_BJT 9     // Hardware pin 12. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define ELEVATOR_BJT 10     // Hardware pin 27. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define WAKEUP_PIN 11       // Hardware pin 25. Pin that attaches to the EXTINT of ublox module.
#define FALSE_WAKEUP_PIN 12 // Hardware pin 28. Pin that is used for the interrupt that puts the glider into deep sleep. Do not connect anything.
#define LED 13              // Hardware pin 26. Pin that is connected to the main green LED.
#define ERR_LED A0          // Hardware pin 3. Pin that connects ot the red error LED.
#define BAT_VOLTAGE_PIN A1  // Hardware pin 7. Pin for battery voltage measurement.
#define WRITE_PIN A2        // Hardware pin 8. Pin that is driven high to receive SerialUSB data from the ATMega.
// #define GPIO A3 // Hardware pin 9. Unused GPIO on autopilot.
#define WIRELESS_RX A4 // Hardware pin 10. Pin that connects to the HCO5. More info in the docs.
#define WIRELESS_TX A5 // Hardware pin 47. Pin that connects to the HCO5. More info in the docs.

// Other settings.
#define SERIAL_BAUD_RATE 115200 // SerialUSB monitor baud rate.
#define EEPROM_BUTTON           // If the data from the main autopilot was collected with the EEPROM_BUTTON mode enabled, enable EEPROM_BUTTON mode here.
// #define SPREADSHEET             // If enabled, only essential serial messages are printed for seamless integration into spreadsheets.

#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <Wire.h>

long startTimer, address, length;

int yaw, pitch, temp, pressure;

bool readEEPROM = false;

ExternalEEPROM myMem;

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

  Wire.begin();

  blink(LED);

  SerialUSB.begin(SERIAL_BAUD_RATE);
  while (!SerialUSB)
    ;

#ifndef SPREADSHEET
  SerialUSB.println("StratoSoar MK2.x EEPROM reader.");
#endif

  myMem.setMemoryType(512); // Valid types: 0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1025, 2048

  if (myMem.begin() == false) {
    SerialUSB.println("No EEPROM detected. Freezing sketch.");
    while (1)
      blink(ERR_LED);
  }

#ifndef SPREADSHEET
  SerialUSB.println("EEPROM detected!");
  SerialUSB.print("EEPROM size in bytes: ");
  length = myMem.length();
  SerialUSB.println(length);

  byte previous = myMem.read(0);
  SerialUSB.println("Testing EEPROM.");
  SerialUSB.print("I read (should be 200): ");
  myMem.write(0, 200);
  SerialUSB.println(myMem.read(0));
  myMem.write(0, previous);

  SerialUSB.println("Beginning reading in 10 seconds...");
#ifndef EEPROM_BUTTON
  SerialUSB.println("Yaw, pitch, temperature (C), pressure (hPa), humidity, voltage");
#endif
#endif

  delay(10000);
  startTimer = millis();
}

void loop() {
  if (address < length) {
#ifndef EEPROM_BUTTON
    // SerialUSB.print("Yaw: ");
    SerialUSB.print(int(myMem.read(address)) * 2);
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
    // SerialUSB.print(", Pitch: ");
    SerialUSB.print(", ");
    SerialUSB.print(myMem.read(address));
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
    // SerialUSB.print(", Temp: ");
    SerialUSB.print(", ");
    SerialUSB.print(myMem.read(address));
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
    // SerialUSB.print(", Pressure: ");
    SerialUSB.print(", ");
    SerialUSB.print(int(myMem.read(address)) * 500);
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
    // SerialUSB.print(", Humidity: ");
    SerialUSB.print(", ");
    SerialUSB.print(int(myMem.read(address)));
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
    // SerialUSB.print(", Voltage: ");
    SerialUSB.print(", ");
    SerialUSB.println(int(myMem.read(address)));
    address++;
    while (myMem.isBusy()) {
      delayMicroseconds(100);
    }
#endif
#ifdef EEPROM_BUTTON
    yaw = int(myMem.read(address));
    address++;
    pitch = int(myMem.read(address));
    address++;
    temp = int(myMem.read(address));
    address++;
    pressure = int(myMem.read(address));
    address++;
    readEEPROM = true;
    if ((yaw == pitch) && (yaw == temp) && (yaw == pressure)) {
      if (yaw != 0) {
        SerialUSB.print("Flight number: ");
        SerialUSB.println(yaw);
        SerialUSB.println("Yaw, Pitch, Temperature (C), Pressure (hPa), Humidity, Voltage, Turning angle, Rudder position, Latitude,    Longitude");
      }
      readEEPROM = false;
    }
    if (readEEPROM) {
      // SerialUSB.print("Yaw: ");
      SerialUSB.print(yaw * 2);
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Pitch: ");
      SerialUSB.print(",   ");
      SerialUSB.print(pitch);
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Temp: ");
      SerialUSB.print(",    ");
      SerialUSB.print(temp);
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Pressure: ");
      SerialUSB.print(",              ");
      SerialUSB.print(pressure * 5);
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Humidity: ");
      SerialUSB.print(",           ");
      SerialUSB.print(int(myMem.read(address)));
      address++;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Voltage: ");
      SerialUSB.print(",        ");
      SerialUSB.print(int(myMem.read(address)));
      address++;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Turning angle: ");
      SerialUSB.print(",      ");
      SerialUSB.print(int(myMem.read(address)) * 2);
      address++;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Rudder position: ");
      SerialUSB.print(",            ");
      SerialUSB.print(int(myMem.read(address)));
      address++;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Lat: ");
      SerialUSB.print(",             ");
      SerialUSB.print(float(readFloatFromEEPROM(address)), 6);
      address = address + 4;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }

      // SerialUSB.print(", Lon: ");
      SerialUSB.print(",   ");
      SerialUSB.println(float(readFloatFromEEPROM(address)), 6);
      address = address + 4;
      while (myMem.isBusy()) {
        delayMicroseconds(100);
      }
    } else {
      // SerialUSB.println("0, 0, 0, 0, 0, 0, 0, 0, 0, 0");
      address = address + 12;
    }
#endif
  } else {
#ifndef SPREADSHEET
    SerialUSB.print("Time took to check all addresses in seconds: ");
    SerialUSB.println((millis() - startTimer) / 1000);
#endif
    while (1)
      ;
  }
}

void blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
  delay(500);
  digitalWrite(pin, HIGH);
  delay(500);
  digitalWrite(pin, LOW);
  delay(500);
}

float readFloatFromEEPROM(int address) {
  float value;
  byte *p = (byte *)(void *)&value; // Pointer to the float value
  for (int i = 0; i < sizeof(value); i++) {
    *p++ = myMem.read(address + i);
  }
  return value;
}