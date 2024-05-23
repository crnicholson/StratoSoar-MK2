// WARNING:
// This is still a work in progress! Code may not work!

/*
autopilot.ino, part of StratoSoar MK2, for an autonomous glider.
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

// https://github.com/crnicholson/StratoSoar-MK2/.

#include "SparkFun_External_EEPROM.h" // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
#include <ArduinoLowPower.h>
#include <Servo.h>
#include <SparkFun_u-blox_GNSS_v3.h> // http://librarymanager/All#SparkFun_u-blox_GNSS_v3.
#include <Wire.h>

#include "src/settings.h"
#include "src/utils.h"

#define pi 3.14159265358979323846

ExternalEEPROM eeprom;

int yawDifference, nowEEPROM, flightNumber, cycles, previous;
float writeTime;
long lastEEPROM = 123456;
int lastYaw = 361;
bool spiral = false;
bool runEEPROM = true;
bool fastUpdatePeriod = true;
bool inFastEEPROM = false;
long eepromAddress, startTimer, endTimer, eepromSize, averageWrite, now, ms, last, nowGPS, msGPS, lastGPS;

// Setpoint and input variables.
double setpointRudder = 0.0; // Desired turn angle (in degrees) this is just a random value for now, the code will change it.
double inputRudder = 0.0;

// Variables for PID control.
double errorRudder = 0.0;     // Error (difference between setpoint and input).
double prevErrorRudder = 0.0; // Previous error.
double integralRudder = 0.0;  // Integral of the error.

// Input variable.
double inputElevator = 0.0; // Current measured pitch angle, this is gibberish right now, and it will be changed by code.

// Variables for PID control.
double errorElevator = 0.0;     // Error (difference between setpoint and input).
double prevErrorElevator = 0.0; // Previous error.
double integralElevator = 0.0;  // Integral of the error.

struct __attribute__((packed)) dataStruct {
  float lat;
  float lon;
  int alt;
  int sats;
  int fixType;
  int speed;
  int seconds;
  int minutes;
  int hours;
  int day;
  int month;
  int year;
  float temp;     // In Celsius.
  float pressure; // In hPa.
  float humidity; // Relative humidity.
  int yaw;
  int pitch;
  int roll;
  int servoPositionElevator;
  int servoPositionRudder;
  float volts;
  float turnAngle;
  float distanceMeters;
  float hdop;
} data;

struct __attribute__((packed)) dataStructIMU {
  float pitch;
  float roll;
  float yaw;
  long temp;
  int32_t pressure;
  long humidity;
} receivedData;

SFE_UBLOX_GNSS gps; // Init GPS.

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
  pinMode(BUTTON, INPUT_PULLUP);
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

  longPulse(LED, 0); // Pulse LED to show power up.
  longPulse(ERR_LED, 0);

#ifdef DEVMODE
  SerialUSB.begin(SERIAL_BAUD_RATE); // Start the serial monitor.
  while (!SerialUSB) {
    longPulse(ERR_LED, 0); // Wait for serial to connect
  }

  delay(1000);

  SerialUSB.println("StratoSoar MK2.x Flight Controller");
#endif

  Serial1.begin(BAUD_RATE); // Hardware serial connection to the ATMega and the IMU.

  longPulse(LED, 0);
  longPulse(ERR_LED, 0);

#ifdef DEVMODE
  I2CScan(); // This will long pulse LED if there is an error.
#endif

#ifdef USE_EEPROM
  eeprom.setMemoryType(512); // Valid types: 0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1025, 2048

  if (eeprom.begin() == false) {
#ifdef DEVMODE
    SerialUSB.println("No EEPROM detected. Freezing sketch.");
#endif
    while (1)
      longPulse(ERR_LED, 0);
  }

  eepromSize = eeprom.length();
#ifdef DEVMODE
  SerialUSB.println("EEPROM detected!");
  SerialUSB.print("EEPROM size in bytes: ");
  SerialUSB.println(eepromSize);
#endif
#ifdef ERASE_EEPROM
#ifdef DEVMODE
  SerialUSB.println("Erasing EEPROM, should take ~10 seconds.");
#endif
  startTimer = millis();
  eeprom.erase();
  endTimer = millis();
#ifdef DEVMODE
  SerialUSB.print("Done erasing. Time took in milliseconds: ");
  SerialUSB.println(endTimer - startTimer);
#endif
#endif

  /*
  #ifdef DEVMODE
    SerialUSB.println("Testing write time.");
  #endif
    averageWrite = eeprom.detectWriteTimeMs(10);
  #ifdef DEVMODE
    SerialUSB.print("Average write time in milliseconds: ");
    SerialUSB.println(averageWrite);
  #endif
  */

  // Testing EEPROM.
  eeprom.write(0, 200);

#ifdef DEVMODE
  SerialUSB.println("Testing EEPROM.");
  SerialUSB.print("EEPROM reads (should be 200): ");
  SerialUSB.println(eeprom.read(0));
#endif
  if (eeprom.read(0) != 200) {
#ifdef DEVMODE
    SerialUSB.println("Warning: EEPROM is not working as expected. Try removing the average write time finder from the code.");
#endif
    longPulse(ERR_LED, 0);
  }
#endif

#ifdef NEED_RUDDER
  rudderServo.attach(RUDDER_PIN);
#endif
#ifdef NEED_ELEVATOR
  elevatorServo.attach(ELEVATOR_PIN);
#endif
#ifdef NEED_PARACHUTE
  parachute.attach(PARACHUTE_PIN);
#endif

#ifdef NEED_RUDDER
  moveRudder(90, 0); // Move the rudder to 90 degrees.
#endif
  delay(1000);
#ifdef NEED_ELEVATOR
  moveElevator(90, 0); // Move the elevator to 90 degrees.
#endif

#ifndef TEST_COORD
  if (gps.begin() == false) { // Connect to the u-blox module using Wire port.
#ifdef DEVMODE
    SerialUSB.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
#endif
    while (1)
      longPulse(ERR_LED, 0);
  }

  // gps.hardReset(); // Hard reset - force a cold start.

  gpsConfig(); // This will long pulse LED if there is an error.

  waitForFix();
#endif

  delay(1000);

#ifdef DEVMODE
  SerialUSB.println("Everything has initialized and the script starts in 10 seconds.");
#endif

  delay(10000);

  writeTime = (FLIGHT_TIME * 60) / (eepromSize / 16);

  startTimer = millis();
  last = startTimer;
}

void loop() {
  shortPulse(LED);
#ifndef TEST_COORD
#ifdef LOW_POWER
  if (!fastUpdatePeriod || !spiral) {
    nowGPS = millis();
    msGPS = nowGPS - lastGPS;
    if (msGPS > GPS_SLEEP) {
      gpsWakeup();  // Wakeup GPS.
      waitForFix(); // Wait for a fix to get data from the GPS, and put the received data into the struct.
      lastGPS = millis();

      // powerOff uses the 8-byte version of RXM-PMREQ - supported by older (M8) modules, like so:
      // gps.powerOff(sleepForSecs * 1000);

      // powerOffWithInterrupt uses the 16-byte version of RXM-PMREQ - supported by the M10 etc. powerOffWithInterrupt allows us to set the force flag.
      // The M10 integration manual states: "The "force" flag must be set in UBX-RXM-PMREQ to enter software standby mode."
      gps.powerOffWithInterrupt(60000, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, true); // No (additional) wakeup sources. force = true.
    }
  } else {
    // waitForFix();
    getGPSData();
  }
#endif
#ifndef LOW_POWER
  nowGPS = millis();
  msGPS = nowGPS - lastGPS;
#ifdef GROUND
  if (msGPS > 15000 && !inFastEEPROM) {
    lastGPS = millis();
    getGPSData();
  }
#endif
#ifndef GROUND
  if (msGPS > GPS_SLEEP && !inFastEEPROM) {
    lastGPS = millis();
    getGPSData();
  }
#endif
#endif
#endif

#ifdef TEST_COORD
  data.lat = testLat;
  data.lon = testLon;
#endif

  getIMUData(); // Get data from the ATMega. This will short pulse LED if there is an error.
  calculate(); // Find distance, turning angle, and more.

#ifdef CHANGE_TARGET
  if ((distanceMeters >= 10000) && (data.alt <= 1000)) {
    targetLat = 42.7, targetLon = -71.9; // Change to random nearby coordinates as a back up location if previous location is too far.
  }
  if ((distanceMeters >= 50000) && (data.alt <= 1000)) {
    targetLat = 43.7, targetLon = -72.9; // Change to random nearby coordinates as a back up location if previous location is too far.
  }
  calculate(); // Calculate again to get updated variables if the target has changed.
#endif

#ifdef SPIN_STOP
  if ((data.distanceMeters <= SPIRAL_DST_THRESHOLD) && (data.alt > SPIRAL_ALT_THRESHOLD)) {
    spiral = true;
#ifdef NEED_RUDDER
    moveRudder(145); // Sends into a spin to safely make its way down.
#endif
    // Once spiraling, skip the main sketch and only wakeup every 5 seconds to see if it's time to open the parachute.
  }
#endif

#ifdef NEED_PARACHUTE
  if ((distanceMeters <= PARACHUTE_DST_THRESHOLD) && (data.alt <= PARACHUTE_ALT_THRESHOLD)) {
    parachute.write(90); // Open the parachute under 500 feet to land.
    // Once the parachute is open, this script skips over the moving servos function and instead goes to an infinite sleep.
    gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, true); // Only wakeup with an interrupt (I think).
    LowPower.attachInterruptWakeup(FALSE_WAKEUP_PIN, onWake, CHANGE);
    LowPower.deepSleep();
  }
#endif

  if (lastYaw != 361) {
    yawDifference = lastYaw - data.yaw;
  } else {
    yawDifference = YAW_DFR_THRESHOLD + 1;
  }

#ifdef DEVMODE
  if (yawDifference < YAW_DFR_THRESHOLD) {
    SerialUSB.println("Within threshold.");
    displayData();
    /*
#ifdef LOW_POWER
    LowPower.sleep(25);
#endif
#ifndef LOW_POWER
    delay(25);
#endif
*/
  }
#endif

  // If the glider is in the initial fast-update period of operation, not spiraling down, or the heading drift is greater than the threshold, move the servos.
  if (!spiral) {
    if (fastUpdatePeriod || abs(yawDifference) > YAW_DFR_THRESHOLD) {
      shortPulse(LED); // Pulse LED to show we are running.

#ifdef DEVMODE
      if (fastUpdatePeriod) {
        SerialUSB.print("In fast-update period with ");
        SerialUSB.print(FAST_UPDATE_PERIOD - (ms / 1000.00));
        SerialUSB.println(" seconds left.");
      } else {
        SerialUSB.println("Out of threshold.");
        displayData();
      }
      /*
      #ifdef LOW_POWER
            LowPower.sleep(25);
      #endif
      #ifndef LOW_POWER
            delay(25);
      #endif
      */
#endif

#ifdef NEED_RUDDER
      moveRudder(data.servoPositionRudder);
#endif
#ifdef NEED_ELEVATOR
      /*
          #ifdef LOW_POWER
                LowPower.sleep(100);
          #endif
          #ifndef LOW_POWER
                delay(100); // Have a small delay to release the draw on the power supply.
          #endif
          */

      moveElevator(data.servoPositionElevator);
#endif

      now = millis();
      ms = now - startTimer;
      lastYaw = data.yaw;
      if (ms < (FAST_UPDATE_PERIOD * 1000)) { // Check if it is still in the fast update period.
#ifndef GROUND
#ifdef LOW_POWER
        LowPower.sleep(ABV_THRS_FST_UPDT_SLPFRST_FVE_SLP - 200);
#endif
#ifndef LOW_POWER
        delay(ABV_THRS_FST_UPDT_SLPFRST_FVE_SLP - 200);
#endif
#endif
        fastUpdatePeriod = true;
      } else {
#ifdef LOW_POWER
        LowPower.sleep(ABOVE_THRESHOLD_SLEEP - 200);
#endif
#ifndef LOW_POWER
        delay(ABOVE_THRESHOLD_SLEEP - 200);
#endif
        fastUpdatePeriod = false;
      }
#ifdef DIVE_STALL
      if (TOO_SLOW <= 5) {
        elevatorServo.write(115); // Dive down when stalled.
      }
#endif
#ifdef USE_EEPROM
#ifndef EEPROM_BUTTON
      if (eepromAddress < eepromSize) {
        if ((millis() - lastEEPROM) > writeTime) {
          lastEEPROM = millis();
          if (sizeof(byte(data.yaw / 2)) == 1) {
            eeprom.write(eepromAddress, byte(data.yaw / 2));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          if (sizeof(byte(abs(data.pitch))) == 1) {
            eeprom.write(eepromAddress, byte(abs(data.pitch)));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          if (sizeof(byte(abs(data.temp))) == 1) {
            eeprom.write(eepromAddress, byte(abs(data.temp)));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          if (sizeof(byte(data.pressure / 5)) == 1) {
            eeprom.write(eepromAddress, byte(data.pressure / 5));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          if (sizeof(byte(data.humidity)) == 1) {
            eeprom.write(eepromAddress, byte(data.humidity));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          if (sizeof(byte(data.volts)) == 1) {
            eeprom.write(eepromAddress, byte(data.volts));
          } else {
            eeprom.write(eepromAddress, 255);
          }
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
        }
      }
#endif
#ifdef EEPROM_BUTTON
      if (!digitalRead(BUTTON) && !inFastEEPROM) {
        flightNumber++;
#ifdef NEED_RUDDER
        previous = rudderServo.read();
        moveRudder(0);
        delay(500);
        moveRudder(180);
        delay(500);
        moveRudder(previous);
#endif
#ifdef DEVMODE
        SerialUSB.print("Writing to what should be the yaw spot on the EEPROM with an address of ");
        SerialUSB.print(eepromAddress);
        SerialUSB.print(" and a flight number of ");
        SerialUSB.println(flightNumber);
#endif
        eeprom.write(eepromAddress, flightNumber);
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }
        eeprom.write(eepromAddress, flightNumber);
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }
        eeprom.write(eepromAddress, flightNumber);
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }
        eeprom.write(eepromAddress, flightNumber);
        eepromAddress = eepromAddress + 13;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }
        cycles = 0;
        inFastEEPROM = true;
      }
      if (cycles <= EEPROM_CYCLES && inFastEEPROM) {
#ifdef DEVMODE
        SerialUSB.print("Writing to what should be the yaw spot on the EEPROM with an address of ");
        SerialUSB.print(eepromAddress);
        SerialUSB.print(" and a yaw of ");
        SerialUSB.println(byte(data.yaw / 2));
#endif
        // Yaw.
        if (sizeof(byte(data.yaw / 2)) == 1) {
          eeprom.write(eepromAddress, byte(data.yaw / 2));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Pitch.
        if (sizeof(byte(abs(data.pitch))) == 1) {
          eeprom.write(eepromAddress, byte(abs(data.pitch)));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Temperature.
        if (sizeof(byte(abs(data.temp))) == 1) {
          eeprom.write(eepromAddress, byte(abs(data.temp)));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Pressure.
        if (sizeof(byte(data.pressure / 5)) == 1) {
          eeprom.write(eepromAddress, byte(data.pressure / 5));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Humidity.
        if (sizeof(byte(data.humidity)) == 1) {
          eeprom.write(eepromAddress, byte(data.humidity));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Voltage.
        if (sizeof(byte(data.volts)) == 1) {
          eeprom.write(eepromAddress, byte(data.volts));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Turning angle.
        if (sizeof(byte(abs(data.turnAngle / 2))) == 1) {
          eeprom.write(eepromAddress, byte(abs(data.turnAngle / 2)));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Rudder position.
        if (sizeof(byte(abs(data.servoPositionRudder))) == 1) {
          eeprom.write(eepromAddress, byte(abs(data.servoPositionRudder)));
        } else {
          eeprom.write(eepromAddress, 255);
        }
        eepromAddress++;
        while (eeprom.isBusy()) {
          delayMicroseconds(100);
        }

        // Latitude.
        if (sizeof(data.lat) == 4) {
          writeFloatToEEPROM(eepromAddress, data.lat);
          eepromAddress = eepromAddress + 4;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
        } else {
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
        }

        // Longitude.
        if (sizeof(data.lon) == 4) {
          writeFloatToEEPROM(eepromAddress, data.lon);
          eepromAddress = eepromAddress + 4;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
        } else {
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
          eeprom.write(eepromAddress, 255);
          eepromAddress++;
          while (eeprom.isBusy()) {
            delayMicroseconds(100);
          }
        }
        cycles++;
      } else {
        inFastEEPROM = false;
      }
#endif
#endif
    } else {
#ifdef LOW_POWER
      LowPower.sleep(BELOW_THRESHOLD_SLEEP); // If the heading drift is below the threshold, sleep and repeat the cycle until the heading drift is above threshold.
#endif
#ifndef LOW_POWER
      delay(BELOW_THRESHOLD_SLEEP); // If the heading drift is below the threshold, sleep and repeat the cycle until the heading drift is above threshold.
#endif
    }
  } else {
#ifdef LOW_POWER
    LowPower.sleep(SPIRAL_SLEEP); // If spiraling, skip above section and wakeup every "SPIRAL_SLEEP" ms only to check GPS to see if it's time to open the parachute.
#endif
#ifndef LOW_POWER
    delay(SPIRAL_SLEEP); // If spiraling, skip above section and wakeup every "SPIRAL_SLEEP" ms only to check GPS to see if it's time to open the parachute.
#endif
  }
}