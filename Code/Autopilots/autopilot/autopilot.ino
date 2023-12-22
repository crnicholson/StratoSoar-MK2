/*
StratoSoar, an autonomous glider project.
Majortiy of this code was written by myself, Charles Nicholson, 2023. autopilotIMU.vx.x was mostly written by J. Remington.
To use this program, change the values in "settings.h" to your liking. No values in "vars.h" have to be changed.
Then, upload the program "autopilotIMU.vx.x" to a different Arduino.
That Arduino will be hooked up over SoftwareSerialUSB. It will be sending yaw, pitch, temp, and pressure data.
The corresponding data can be used in calculations to move the servos accordingly.
Two Arduinos are used so the master (this one) can go to sleep in the code and save power.
Sleep in the program "autopilotIMU.vx.x" causes catasrpothic failure, but it is already low power enough so sleep is not needed.
MOSFETs are used in this program to turn the servos on and off to save power. The one I use is the 30N06L, a logic level N-Channel FET.
I also use an NPN BJT (2N3906) to power on and off the PWM line.
I do this so when the FET is low, ground can't go through the servo PWM (signal) line, which would damage the servo and the Arduino.

NOTE: For proper functionality, make sure the data transmission rate is more than the reading.
      I.E. sendMs in autopilotIMU.vx.x is equal to 1500 and the combined delay in this sketch is equal to 1000 ms.

NOTE: this error message does not effect the performance of the autopilot: "uint8_t requestFrom(uint8_t, uint8_t);".
*/

// ***** To-Do *****
// Format code - add periods, capitalize, format above documentation
// Write documentation
// Add DEVMODE
// Capitalize #defines
// Change comments
// Update for SAMD
// Make it good

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
  Wire.begin();
  SerialUSB.begin(SERIAL_BAUD_RATE); // Start the serial monitor.
  Serial1.begin(BAUD_RATE);          // Hardware serial connection to the ATMega and the IMU.
  longPulse();                       // Pulse LED to show power up.

  if (needRudder) {
    rudderServo.attach(RUDDER_PIN);
  }
  if (needElevator) {
    elevatorServo.attach(ELEVATOR_PIN);
  }
  if (needParachute) {
    parachute.attach(PARACHUTE_PIN);
  }

  digitalWrite(RUDDER_FET, HIGH); // Turn servo on.
  digitalWrite(RUDDER_BJT, LOW);  // Turn signal line on.
  rudderServo.write(90);
  delay(200);
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
  delay(1000);
  digitalWrite(ELEVATOR_FET, HIGH); // Turn servo on.
  digitalWrite(ELEVATOR_BJT, LOW);  // Turn signal line on.
  elevatorServo.write(90);
  delay(200);
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(ELEVATOR_FET, LOW);
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

    double distanceMeters = calculateDistance(currentLat, currentLon, targetLat, targetLon);

#ifdef CHANGE_TARGET
    if ((distanceMeters >= 10000) && (gps.altitude.feet() <= 1000)) {
      targetLat = 42.7, targetLon = -71.9; // Change to random nearby coordinates as a back up location if previous location is too far
    }

    if ((distanceMeters >= 50000) && (gps.altitude.feet() <= 5000)) {
      targetLat = 43.7, targetLon = -72.9; // Change to random nearby coordinates as a back up location if previous location is too far
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

    if (needParachute) {
      if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 500) {
        parachute.write(90); // Open the parachute under 500 feet to land
        landed = true;
      }
    }
  }

  if (!spiral && !stall) {
    receiveData(); // Get data from the ATMega.

    turnAngle = turningAngle(currentLat, currentLon, yaw, targetLat, targetLon);

    int servoPositionElevator = pidMagicElevator(); // Change PID values in "settings.h" if you want
    int servoPositionRudder = pidMagicRudder();     // Change PID values in "settings.h" if you want

    digitalWrite(RUDDER_FET, HIGH); // Turning on the servo. servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
    digitalWrite(RUDDER_BJT, LOW);  // Turning on the servo PWM line
    rudderServo.write(servoPositionRudder);
    delay(200);
    digitalWrite(RUDDER_BJT, HIGH); // Turning off the servo PWM line
    digitalWrite(RUDDER_FET, LOW);  // Turning off the servo
    delay(600);
    digitalWrite(ELEVATOR_FET, HIGH); // Turning on the servo. servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
    digitalWrite(ELEVATOR_BJT, LOW);  // Turning on the servo PWM line
    rudderServo.write(servoPositionElevator);
    delay(200);
    digitalWrite(ELEVATOR_BJT, HIGH); // Turning off the servo PWM line
    digitalWrite(ELEVATOR_FET, LOW);  // Turning off the servo
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

    if (needParachute) {
      if (speedMPH <= 5) {
        elevatorServo.write(115); // Dive down when stalled
      }
    }

    // This saves to an external EEPROM (AT24Cx) so we can get some yummy data
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
  }
}