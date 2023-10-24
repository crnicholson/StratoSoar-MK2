/*
StratoSoar, an autonomous glider project.
Majortiy of this code was written by myself, Charles Nicholson, 2023. autopilotIMU.vx.x was mostly written by J. Remington.
To use this program, change the values in "settings.h" to your liking.
Then, upload the program "autopilotIMU.vx.x" to a different Arduino.
That Arduino will be hooked up over SoftwareSerial. It will be sending yaw, pitch, temp, and pressure data.
The corresponding data can be used in calculations to move the servos accordingly. 
Two Arduinos are used so the master (this one) can go to sleep in the code and save power.
Sleep in the program "autopilotIMU.vx.x" causes catasrpothic failure, but it is already low power enough so sleep is not needed.
MOSFETs are used in this program to turn the servos on and off to save power. The one I use is the 30N06L, a logic level N-Channel FET.
I also use an NPN BJT (2N3906) to power on and off the PWM line. 
I do this so when the FET is low, ground can't go through the servo PWM (signal) line, which would damage the servo and the Arduino.

NOTE: For proper functionality, make sure the data transmission rate is more than the reading. 
      I.E. sendMs (in autopilotIMU.vx.x) = 1500 and the combined delay in this sketch is equal to 1000 ms.

NOTE: this error message does not effect the performance of the autopilot: "uint8_t requestFrom(uint8_t, uint8_t);".
*/

// #include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <ServoTimer2_8mhz.h> // Other servo library interferes with SW Serial
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <Narcoleptic.h>
#include "settings.h" // File with settings for the autopilot, change this instead of the code
#include "vars.h" // File with most of the variables

// Creating some objects:
// TinyGPSPlus gps;
// SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial mcuConn(mcuRX, mcuTX);

void setup() {
  /*
  Narcoleptic.disableSPI(); // Wasted power
  
  // Another tweak to lower the power consumption
  ADCSRA &= ~(1<<ADEN); // Disable ADC
  ACSR = (1<<ACD); // Disable the analog comparator
  */
  pinMode(13, OUTPUT);
  pinMode(rudderBJT, OUTPUT);
  pinMode(elevatorBJT, OUTPUT);
  pinMode(rudderFET, OUTPUT);
  pinMode(elevatorFET, OUTPUT);
  Wire.begin();
  Serial.begin(SerialBaud); // Start the Serial communications
  mcuConn.begin(MCUBaud);
  // ss.begin(GPSBaud);
  
  if (needRudder) {
    rudderServo.attach(rudderPin);
  }
  if (needElevator) {
    elevatorServo.attach(elevatorPin);
  }
  if (needParachute) {
    parachute.attach(parachutePin);
  }

  digitalWrite(rudderFET, HIGH); // Turn servo on
  digitalWrite(rudderBJT, LOW); // Turn signal line on
  rudderServo.write(1550); // Equal to 90 degrees
  delay(200);
  digitalWrite(rudderBJT, HIGH);
  digitalWrite(rudderFET, LOW);
  delay(1000);
  digitalWrite(elevatorFET, HIGH); // Turn servo on
  digitalWrite(elevatorBJT, LOW); // Turn signal line on
  elevatorServo.write(1550); // Equal to 90 degrees
  delay(200);
  digitalWrite(elevatorBJT, HIGH);
  digitalWrite(elevatorFET, LOW);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  Serial.println("Everything has initialized and the script starts in 1 second!");
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

    currentLat = testLat;
    currentLon = testLon;

    // double distanceMeters = gps.distanceBetween(currentLat, currentLon, targetLat, targetLon);

    /*
    if ((distanceMeters >= 10000) && (gps.altitude.feet() <= 1000)) {
      targetLat = 42.7, targetLon = -71.9; // Change to random nearby coordinates as a back up location if previous location is too far
    }

    if ((distanceMeters >= 50000) && (gps.altitude.feet() <= 5000)) {
      targetLat = 43.7, targetLon = -72.9; // Change to random nearby coordinates as a back up location if previous location is too far
    }
   */

    /*
    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() >= 5000) {
      spiral = true;
      rudderServo.write(145); // Sends into a spin to safely make it's way down   
    }

    if ((distanceMeters <= 100) && gps.location.isValid() && gps.altitude.feet() <= 500) {
      parachute.write(90); // Open the parachute under 500 feet to land 
      landed = true;
    }
    */

    if (!spiral) {
      if (mcuConn.available() >= 6) { // Check to see how many bytes we have to read
        byte yawReceive = mcuConn.read(); // Read the transmitted bytes from autopilotIMU.vx.x
        byte pitchReceive = mcuConn.read();
        byte negativePitch = mcuConn.read();
        byte tempReceive = mcuConn.read();
        byte negativeTemp = mcuConn.read();
        byte pressureReceive = mcuConn.read();

        yaw = int(yawReceive) * 2;  // Convert the received byte back to an integer
        pitch = int(pitchReceive);  // Convert the received byte back to an integer
        temp = int(tempReceive);  // Convert the received byte back to an integer
        pressure = int(pressureReceive) * 500;  // Convert the received byte back to an integer

        if (negativePitch == 1) { // Making some things negative if needed
          pitch = pitch * -1;
        }

        if (negativeTemp == 1) {
          temp = temp * -1;
        }
      }

      turnAngle = turningAngle(currentLat, currentLon, yaw, targetLat, targetLon);

      int servoPositionElevator = pidMagicElevator(); // Change PID values in "settings.h" if you want
      int servoPositionRudder = pidMagicRudder(); // Change PID values in "settings.h" if you want

      digitalWrite(rudderFET, HIGH); // Turning on the servo. servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
      digitalWrite(rudderBJT, LOW); // Turning on the servo PWM line
      rudderServo.write(servoPositionRudder);
      delay(200);
      digitalWrite(rudderBJT, HIGH); // Turning off the servo PWM line
      digitalWrite(rudderFET, LOW); // Turning off the servo
      // Turn GPS off
      Narcoleptic.delay(600); // Low power mode. Saves about 3 mA. 
      // Turn GPS on
      digitalWrite(elevatorFET, HIGH); // Turning on the servo. servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
      digitalWrite(elevatorBJT, LOW); // Turning on the servo PWM line
      rudderServo.write(servoPositionElevator);
      delay(200);
      digitalWrite(elevatorBJT, HIGH); // Turning off the servo PWM line
      digitalWrite(elevatorFET, LOW); // Turning off the servo
      Serial.print("Turning Angle: ");
      Serial.print(turnAngle);
      Serial.print(", Heading: ");
      Serial.print(yaw);
      Serial.print(", Pitch: ");
      Serial.print(pitch);
      Serial.print(", Rudder Servo position: ");
      Serial.print(servoPositionRudderDegrees);
      Serial.print(", Elevator Servo position: ");
      Serial.println(servoPositionElevatorDegrees);

      // speedMPH = gps.speed.mph(); // GPS speed (idk how accurate it is, though it probably is when moving fast-ish in a straight manner)

      /*
      if (speedMPH <= 5) {
        elevatorServo.write(115); // Dive down when stalled
      }
      */
    
      // This saves to an external EEPROM (AT24Cx) so we can get some yummy data
      if (runEEPROM) { 
        writeToEEPROM(eepromI2CAddress, eepromAddress, int(yaw / 2));
        delay(10);
        eepromAddress++;  
        writeToEEPROM(eepromI2CAddress, eepromAddress, pitch); 
        delay(10);
        eepromAddress++;  
        writeToEEPROM(eepromI2CAddress, eepromAddress, int(temp));
        delay(10);
        eepromAddress++;
        writeToEEPROM(eepromI2CAddress, eepromAddress, int(pressure/500)); 
        delay(10);
        eepromAddress++;
        if (eepromAddress >= maxAddress) {
          runEEPROM = false;
        }
      }
    } 
  } 
}

// loop is now over, on to the boring functions that nobody cares about

double deg2rad(double deg) {
  return deg * pi / 180.0;
}

double rad2deg(double rad) {
  return rad * 180.0 / pi;
}

double turningAngle(double currentLat, double currentLon, double heading1, double targetLat, double targetLon) {
  double a = azimuth(currentLat, currentLon, targetLat, targetLon);
  double h = deg2rad(heading1);
  double angle = a - h;
  if (angle < -pi) angle += 2 * pi;
  if (angle > pi) angle -= 2 * pi;
  return rad2deg(angle);
}

double azimuth(double currentLat, double currentLon, double targetLat, double targetLon) {
  double dLon = deg2rad(targetLon - currentLon);
  double dPhi = log(tan(deg2rad(targetLat) / 2 + pi / 4) / tan(deg2rad(currentLat) / 2 + pi / 4));
  if (fabs(dLon) > pi) {
      dLon = dLon > 0 ? -(2 * pi - dLon) : (2 * pi + dLon);
  }
  return fmod((atan2(dLon, dPhi) + 2 * pi), (2 * pi));
}

void writeToEEPROM(byte EEPROMAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(EEPROMAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
  delay(5);
}

int pidMagicElevator() {
  // Calculate the error
  errorElevator = setpointElevator - pitch; 

  // Calculate the outputElevator
  int outputElevator = KpElevator * errorElevator + KiElevator * integralElevator + KdElevator * (errorElevator - prevErrorElevator); // PID math wizardy

  // Apply the outputElevator to the servo
  servoPositionElevatorDegrees = 90 - outputElevator;  // Adjust servo position based on the outputElevator

  // Update previous error and integral
  prevErrorElevator = errorElevator;
  integralElevator += errorElevator;

  int servoPositionElevatorNew = map(servoPositionElevatorDegrees, 0, 180, 750, 2250); // 750, 2250

  return servoPositionElevatorNew;
}

int pidMagicRudder() {
  setpointRudder = yaw + turnAngle;
  inputRudder = yaw;

  // Calculate the error
  errorRudder = setpointRudder - yaw;

  // Calculate the control output
  int outputRudder = KpRudder * errorRudder + KiRudder * integralRudder + KdRudder * (errorRudder - prevErrorRudder);

  // Apply the control output to the servo
  servoPositionRudderDegrees = 90 + outputRudder;  // Adjust servo position based on the output

  // Update previous error and integral
  prevErrorRudder = errorRudder;
  integralRudder += errorRudder;

  int servoPositionRudderNew = map(servoPositionRudderDegrees, 0, 180, 1250, 1750); // 750, 2250

  return servoPositionRudderNew;
}
