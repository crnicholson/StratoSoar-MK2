// Part of StratoSoar MK2.
// autopilot.ino.
// Charles Nicholson, 2023.
// Code for a low-power autonomous glider.
// https://github.com/crnicholson/StratoSoar-MK2/.

// ***** NOTE *****
// For proper functionality, make sure the data transmission rate is more than the reading.
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
// Maybe implement a voltage reader?

#include "headers/settings.h" // File with settings for the autopilot, change this instead of the code.
#include <Servo.h>
#include <SparkFun_u-blox_GNSS_v3.h> // http://librarymanager/All#SparkFun_u-blox_GNSS_v3.
#include <Wire.h>

#define pi 3.14159265358979323846

int eepromAddress;
bool spiral = false;
bool stall = false;
bool landed = false;
bool runEEPROM = true;

// Setpoint and input variables.
double setpointRudder = 0; // Desired turn angle (in degrees) this is just a random value for now, the code will change it.
double inputRudder = 0.0;  // Current measured turn angle, also gibberish.

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
  int temp;
  int pressure;
  int yaw;
  int pitch;
  int servoPositionElevator;
  int servoPositionRudder;
  int volts;
  float turnAngle;
} data;

SFE_UBLOX_GNSS gps; // Init GPS.

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

  if (gps.begin() == false) { // Connect to the u-blox module using Wire port.
#ifdef DEVMODE
    SerialUSB.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
#endif
    while (1)
      ;
  }

  // gps.hardReset(); // Hard reset - force a cold start.

#ifndef TEST_COORD
  waitForFix();
#endif

  // Following commented out code allows for use of PSMOO (or PSMCT with some edits).
  // PSMOO allows for predefined periodic wakeup of the GPS receiver to get a fix, then go into backup mode.
  // PSMCT allows for the GPS to get a fix, then go into Power Efficient Tracking (POT) state. This is good for trackers that transmit a lot.
  // To use either PSMCT or PSMOO, you can remove the wait-for-fix code and the software backup part. Uncomment the lines below, too.
  // You may need to do more editing of the code to achieve your desired outcome, as this has not been tested yet.
  /*
    gpsConfig();

    uint8_t PSM;

    if (gps.getVal8(UBLOX_CFG_PM_OPERATEMODE, &PSM) ==
        true) { // Test if the GPS config worked correctly.
  #ifdef DEVMODE
      if (PSM == 1) {
        SerialUSB.println("Power save mode set correctly!");
      } else {
        SerialUSB.println("Power save mode configuration failed!");
      }
    } else {
      SerialUSB.println("VALGET failed!");
    }
  */

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
      rudderServo.write(145); // Sends into a spin to safely make it's way down.
    }
#endif

#ifdef STALL_STOP
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 600) {
      stall = true;
      elevatorServo.write(145); // Sends into a spin to safely make it's way down.
    }
#endif

#ifdef NEED_PARACHUTE
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 500) {
      parachute.write(90); // Open the parachute under 500 feet to land
      landed = true;
    }
#endif
  }

  if (!spiral | !stall) {
    receiveData(); // Get data from the ATMega, put it in the data struct.
    getGPSData();  // Get data from the GPS, put it in the data struct.
    calculate();   // Calculate the turning angle and the servo positions.

    data.turnAngle = turningAngle(data.lat, data.lon, data.yaw, targetLat, targetLon);

    data.servoPositionElevator = pidMagicElevator(); // Change PID values in "settings.h" if you want.
    data.servoPositionRudder = pidMagicRudder();     // Change PID values in "settings.h" if you want.

    moveRudder(servoPositionRudder);     // Move servo and turn it off.
    delay(SLEEP_TIME);                   // Add sleep function here.
    moveElevator(servoPositionElevator); // Move servo and turn it off.
#ifdef DEVMODE
    displayData();
    SerialUSB.print("Turning Angle: ");
    SerialUSB.print(turnAngle);
    SerialUSB.print(", Heading: ");
    SerialUSB.print(yaw);
    SerialUSB.print(", Pitch: ");
    SerialUSB.print(pitch);
    SerialUSB.print(", Rudder Servo position: ");
    SerialUSB.print(servoPositionRudder);
    SerialUSB.print(", Elevator Servo position: ");
    SerialUSB.println(servoPositionElevator);
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