// WARNING: 
// This is still a work in progress! Code may not work!

// Part of StratoSoar MK2.
// autopilot.ino.
// Charles Nicholson, 2023.
// Code for a low-power autonomous glider.
// https://github.com/crnicholson/StratoSoar-MK2/.

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
// Work on the wireless function

#include <ArduinoLowPower.h>
#include <Servo.h>
#include <SparkFun_u-blox_GNSS_v3.h> // http://librarymanager/All#SparkFun_u-blox_GNSS_v3.
#include <Wire.h>
#include "headers/settings.h" // File with settings for the autopilot, change this instead of the code. Has to be after other includes.

#define pi 3.14159265358979323846

int eepromAddress, counter, now, start, ms, last, yawDifference;
int lastYaw = 361;
bool spiral = false;
bool runEEPROM = true;
bool firstFive = true;

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
  float temp; // In Celsius.
  float pressure; // In hPa.
  float humidity; // In relative humidity.
  int yaw;
  int pitch;
  int roll;
  int servoPositionElevator;
  int servoPositionRudder;
  int volts;
  float turnAngle;
  float distanceMeters;
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
#ifdef DEVMODE
  SerialUSB.begin(SERIAL_BAUD_RATE); // Start the serial monitor.
  while (!SerialUSB) {
    ; // Wait for serial to connect
  }
#endif
  Serial1.begin(BAUD_RATE); // Hardware serial connection to the ATMega and the IMU.
  longPulse(LED);           // Pulse LED to show power up.
  longPulse(ERR_LED);

  I2CScan();

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

#ifndef TEST_COORD
  if (gps.begin() == false) { // Connect to the u-blox module using Wire port.
#ifdef DEVMODE
    SerialUSB.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
#endif
    while (1)
      ;
  }

  // gps.hardReset(); // Hard reset - force a cold start.

  gpsConfig();

  waitForFix();
#endif

  delay(1000);

#ifdef DEVMODE
  SerialUSB.println("Everything has initialized and the script starts in 20 seconds!");
#endif
  delay(20000);
  start = millis();
  last = millis();
}

void loop() {
#ifndef TEST_COORD
  if (!firstFive | !spiral) {
    now = millis();
    ms = now - last;
    if (ms > 30000) {
      gpsWakeup();  // Wakeup GPS.
      waitForFix(); // Wait for a fix to get data from the GPS, and put the received data into the struct.
      last = millis();

      // powerOff uses the 8-byte version of RXM-PMREQ - supported by older (M8) modules, like so:
      // gps.powerOff(sleepForSecs * 1000);

      // powerOffWithInterrupt uses the 16-byte version of RXM-PMREQ - supported by the M10 etc. powerOffWithInterrupt allows us to set the force flag.
      // The M10 integration manual states: "The "force" flag must be set in UBX-RXM-PMREQ to enter software standby mode."
      gps.powerOffWithInterrupt(60000, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, true); // No (additional) wakeup sources. force = true.
    }
  } else {
    waitForFix();
  }
#endif

#ifdef TEST_COORD
  data.lat = testLat;
  data.lon = testLon;
#endif

  getIMUData(); // Get data from the ATMega.
  calculate();  // Find distance, turning angle, and more.

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
    moveRudder(145); // Sends into a spin to safely make it's way down.
    // Once spiraling, skip the main sketch and only wakeup every 5 seconds to see if it's time to open the parachute.
  }
#endif

#ifdef NEED_PARACHUTE
  if ((distanceMeters <= PARACHUTE_DST_THRESHOLD) && (data.alt <= PARACHUTE_ALT_THRESHOLD)) {
    parachute.write(90); // Open the parachute under 500 feet to land.
    // Once the parachute is open, this script skips over the moving servos function and instead goes to an infinite sleep.
    gps.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, true); // Only wakeup with an interrupt (I think).
    LowPower.attachInterruptWakeup(FALSE_WAKEUP_PIN, onWake, CHANGE);
    LowPower.deepSleep(); // No way to wakeup now!
  }
#endif

  if (lastYaw != 361) {
    yawDifference = lastYaw - data.yaw;
  } else {
    yawDifference = YAW_DFR_THRESHOLD + 1;
  }

  if (!spiral) {
    if (yawDifference < YAW_DFR_THRESHOLD | firstFive | yawDifference > abs(YAW_DFR_THRESHOLD)) {
      shortPulse(LED); // Pulse LED to show we are running.
#ifdef DEVMODE
      // streamData();
      displayData();
      delay(25);
#endif
      moveRudder(data.servoPositionRudder); // Move servo and turn it off. Have the sleep in between to make sure there is minimal draw on the power supply.
      now = millis();
      ms = start - now;
      lastYaw = data.yaw;
      if (ms < 300000) {
        // LowPower.deepSleep(500);
        delay(500);
        firstFive = true;
      } else {
        LowPower.deepSleep(500);
        firstFive = false;
      }
      moveElevator(data.servoPositionElevator); // Move servo and turn it off.
#ifdef DIVE_STALL
      if (TOO_SLOW <= 5) {
        elevatorServo.write(115); // Dive down when stalled.
      }
#endif
#ifdef USE_EEPROM
      // This saves to an external EEPROM (AT24Cx) so we can get some data.
      if (runEEPROM) {
        writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(data.yaw / 2)); // Making some things a byte to save space.
        delay(10);
        eepromAddress++;
        writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(data.pitch));
        delay(10);
        eepromAddress++;
        writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(data.temp));
        delay(10);
        eepromAddress++;
        writeToEEPROM(EEPROM_I2C_ADDRESS, eepromAddress, int(data.pressure / 500));
        delay(10);
        eepromAddress++;
        if (eepromAddress >= MAX_ADDRESS) {
          runEEPROM = false;
        }
      }
#endif
    } else {
      LowPower.deepSleep(500); // If the heading drift is below the threshold, sleep for 500 ms and repeat the cycle until the heading drift is above threshold.
    }
  } else {
    LowPower.deepSleep(500); // If spiraling, skip above section and wakeup every 500 ms only to check GPS to see if it's time to open the parachute.
  }
}