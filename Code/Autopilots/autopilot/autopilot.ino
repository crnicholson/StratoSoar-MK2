// Part of StratoSoar MK2.
// autopilot.ino.
// Charles Nicholson, 2023.
// Code for a low-power autonomous glider.
// https://github.com/crnicholson/StratoSoar-MK2/.

// NOTE: For proper functionality, make sure the data transmission rate is more than the reading.
// I.E. sendMs in autopilotIMU.vx.x is equal to 1500 and the combined delay in this sketch is equal to 1000 ms.

// ***** Usage *****
// 1. Change the values in "settings.h" to your liking. No values in "vars.h" have to be changed.
// 2. Do the same to the program "autopilotIMU.ino".
// 3. Upload the program "autopilot.ino" to the USB on the tracker board. Use the profile "Arduino Zero (Native USB Board)".
// 4. Upload the program "autopilotIMU.ino" to the FTDI port on the tracker board. Use the profile "Arduino Pro Mini, 3.3v, 8MHz".

// ***** Why Two MCUs? ****
// There are two Arduinos (MCUs), an ATMega328P and a SAMD21G18A. The ATMega is sending is data over SoftwareSerial to the hardware serial of the SAMD.
// The reason this is done is because the Mahony program is blocking and is made for AVR microcontrollers.
// Blocking means that it doesn't allow the rest of the sketch to function. Delaying the rest of the sketch prevents low power modes, so that is not good.
// Furthermore, the GPS library that is used requires more memory than what is available on the ATMega, so a SAMD with it's large memory is used.
// The ATMega is low-power enough without sleep modes, so all is well, unlike the SAMD, which takes some more power.

// ***** Calculations and Principles *****
// Once data has been received over serial, calculations to move the servos are made. The first one tis

// ***** Servos *****
// MOSFETs are used in this program to turn the servos on and off to save power.The one I use is the 30N06L, a logic level N - Channel FET.I also use an NPN BJT(2N3906) to power on and off the PWM line.I do this so when the FET is low, ground can't go through the servo PWM (signal) line, which would damage the servo and the Arduino.

// ***** Low-Power *****

// ***** GPS + GPS Low-Power *****

// ***** To-Do *****
// Format code - add periods, capitalize, format above documentation
// Write documentation
// Change comments
// Update for SAMD
// Make it good
// Work on the CHANGE_TARGET to get it smarter
// Get a GPS
// Do GPS low power
// Do GPS config
// Get SAMD low power working
// Add a dump data function
// Send a struct over serial to achieve more accurate data

#include "headers/settings.h" // File with settings for the autopilot, change this instead of the code.
#include "headers/vars.h"     // File with most of the variables.
#include <Servo.h>
#include <Wire.h>

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(RUDDER_BJT, OUTPUT);
  pinMode(ELEVATOR_BJT, OUTPUT);
  pinMode(RUDDER_FET, OUTPUT);
  pinMode(ELEVATOR_FET, OUTPUT);
  digitalWrite(LED, LOW);
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
  digitalWrite(ELEVATOR_FET, LOW);
  Wire.begin();
  SerialUSB.begin(SERIAL_BAUD_RATE); // Start the serial monitor.
  Serial1.begin(BAUD_RATE);          // Hardware serial connection to the ATMega and the IMU.
  longPulse();                       // Pulse LED to show power up.

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
  moveRudder(90); // Move the rudder to 90 degrees.
#endif
  delay(1000);
#ifdef NEED_ELEVATOR
  moveElevator(90); // Move the elevator to 90 degrees.
#endif

#ifdef DEVMODE
  SerialUSB.println("Everything has initialized and the script starts in 1 second!");
#endif
  delay(1000);
}

void loop() {
  if (!landed) {
    /*
    if (gps.location.isValid()) {
      currentLat = gps.location.lat();
      currentLon = gps.location.lng();
    } else {
      currentLat = testLat;
      currentLon = testLon;
    }
    */

#ifdef TEST_COORD
    currentLat = testLat;
    currentLon = testLon;
#endif

    double distanceMeters = calculateDistance(currentLat, currentLon, targetLat, targetLon); // Find the distance between the current location and the target.

#ifdef CHANGE_TARGET
    if ((distanceMeters >= 10000) && (gps.altitude.feet() <= 1000)) {
      targetLat = 42.7, targetLon = -71.9; // Change to random nearby coordinates as a back up location if previous location is too far.
    }

    if ((distanceMeters >= 50000) && (gps.altitude.feet() <= 5000)) {
      targetLat = 43.7, targetLon = -72.9; // Change to random nearby coordinates as a back up location if previous location is too far.
    }
#endif

#ifdef SPIN_STOP
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() >= 5000) {
      spiral = true;
      rudderServo.write(145); // Sends into a spin to safely make it's way down
    }
#endif

#ifdef STALL_STOP
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 600) {
      stall = true;
      elevatorServo.write(145); // Sends into a spin to safely make it's way down
    }
#endif

#ifdef NEED_PARACHUTE
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 500) {
      parachute.write(90); // Open the parachute under 500 feet to land
      landed = true;
    }
#endif
  }

  if (!spiral && !stall) {
    receiveData(); // Get data from the ATMega.

    turnAngle = turningAngle(currentLat, currentLon, yaw, targetLat, targetLon);

    int servoPositionElevator = pidMagicElevator(); // Change PID values in "settings.h" if you want
    int servoPositionRudder = pidMagicRudder();     // Change PID values in "settings.h" if you want

    moveRudder(servoPositionRudder);     // Move servo and turn it off.
    delay(SLEEP_TIME);                   // Add sleep function here.
    moveElevator(servoPositionElevator); // Move servo and turn it off.
#ifdef DEVMODE
    SerialUSB.print("Turning Angle: ");
    SerialUSB.print(turnAngle);
    SerialUSB.print(", Heading: ");
    SerialUSB.print(yaw);
    SerialUSB.print(", Pitch: ");
    SerialUSB.print(pitch);
    SerialUSB.print(", Rudder Servo position: ");
    SerialUSB.print(servoPositionRudderDegrees);
    SerialUSB.print(", Elevator Servo position: ");
    SerialUSB.println(servoPositionElevatorDegrees);
#endif

#ifdef DIVE_STALL
    if (TOO_SLOW <= 5) {
      elevatorServo.write(115); // Dive down when stalled.
    }
#endif

#ifdef USE_EEPROM
    // This saves to an external EEPROM (AT24Cx) so we can get some data.
    if (runEEPROM) {
      writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(yaw / 2));
      delay(10);
      eepromAddress++;
      writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, pitch);
      delay(10);
      eepromAddress++;
      writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(temp));
      delay(10);
      eepromAddress++;
      writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(pressure / 500));
      delay(10);
      eepromAddress++;
      if (eepromAddress >= MAX_ADDRESS) {
        runEEPROM = false;
      }
    }
#endif
  }
}