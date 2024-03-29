/*
utils.ino, part of StratoSoar MK2, for an autonomous glider.
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

#include "headers/settings.h"
#include <Arduino.h>

double toRadians(double deg) {
  return deg * pi / 180.0;
}

double toDegrees(double rad) {
  return rad * 180.0 / pi;
}

double turningAngle(double cLat, double cLon, double head, double tLat, double tLon) {
  double a = azimuth(cLat, cLon, tLat, tLon);
  double h = toRadians(head);
  double angle = a - h;
  if (angle < -pi)
    angle += 2 * pi;
  if (angle > pi)
    angle -= 2 * pi;
  return toDegrees(angle);
}

double azimuth(double cLat, double cLon, double tLat, double tLon) {
  double dLon = toRadians(tLon - cLon);
  double dPhi = log(tan(toRadians(tLat) / 2 + pi / 4) / tan(toRadians(cLat) / 2 + pi / 4));
  if (fabs(dLon) > pi) {
    dLon = dLon > 0 ? -(2 * pi - dLon) : (2 * pi + dLon);
  }
  return fmod((atan2(dLon, dPhi) + 2 * pi), (2 * pi));
}

// Haversine formula to calculate distance between two coordinates.
double calculateDistance(double cLat, double cLon, double tLat, double tLon) {
  double dLat = toRadians(tLat - cLat);
  double dLon = toRadians(tLon - cLon);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(toRadians(tLat)) * cos(toRadians(tLat)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = 6371000 * c;
  return distance;
}

int pidMagicElevator() {
  errorElevator = SETPOINT_ELEVATOR - data.pitch; // Calculate the error.

  int outputElevator = KP_ELEVATOR * errorElevator + KI_ELEVATOR * integralElevator + KD_ELEVATOR * (errorElevator - prevErrorElevator); // Calculate the output.

  int servoPositionElevator2 = 90 - outputElevator; // Adjust servo position based on the output.

  // Update previous error and integral.
  prevErrorElevator = errorElevator;
  integralElevator += errorElevator;

  // int servoPositionElevatorNew = map(servoPositionElevator2, 0, 180, 750, 2250); // For servos that take time not degrees.

  return servoPositionElevator2;
}

int pidMagicRudder() {
  setpointRudder = data.yaw + data.turnAngle;
  inputRudder = data.yaw;

  errorRudder = setpointRudder - data.yaw; // Calculate the error.

  int outputRudder = KP_RUDDER * errorRudder + KI_RUDDER * integralRudder + KD_RUDDER * (errorRudder - prevErrorRudder); // Calculate the output.

  int servoPositionRudder2 = 90 + outputRudder; // Adjust servo position based on the output.

  // Update previous error and integral.
  prevErrorRudder = errorRudder;
  integralRudder += errorRudder;

  // int servoPositionRudderNew = map(servoPositionRudder2, 0, 180, 1250, 1750); // For servos that take time not degrees.

  return servoPositionRudder2;
}

void longPulse(int pin, int sleep = 1) {
  digitalWrite(pin, HIGH);
  if (sleep == 1) {
#ifdef LOW_POWER
    LowPower.sleep(250);
#endif
#ifndef LOW_POWER
    delay(250);
#endif
  } else {
    delay(250);
  }
  digitalWrite(pin, LOW);
  if (sleep == 1) {
#ifdef LOW_POWER
    LowPower.sleep(250);
#endif
#ifndef LOW_POWER
    delay(500);
#endif
  } else {
    delay(500);
  }
  digitalWrite(pin, HIGH);
  if (sleep == 1) {
#ifdef LOW_POWER
    LowPower.sleep(250);
#endif
#ifndef LOW_POWER
    delay(250);
#endif
  } else {
    delay(250);
  }
  digitalWrite(pin, LOW);
}

void shortPulse(int pin) {
  digitalWrite(pin, HIGH);
#ifdef LOW_POWER
  LowPower.sleep(250);
#endif
#ifndef LOW_POWER
  delay(250);
#endif
  digitalWrite(pin, LOW);
}

void moveRudder(int degrees, int sleep = 1) {
  digitalWrite(RUDDER_FET, HIGH); // Turn servo on.
  digitalWrite(RUDDER_BJT, LOW);  // Turn signal line on.
  rudderServo.write(degrees);
  if (sleep == 1) {
#ifdef LOW_POWER
    LowPower.sleep(300);
#endif
#ifndef LOW_POWER
    delay(300);
#endif
  } else {
    delay(300);
  }
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
}

void moveElevator(int degrees, int sleep = 1) {
  digitalWrite(ELEVATOR_FET, HIGH); // Turn servo on.
  digitalWrite(ELEVATOR_BJT, LOW);  // Turn signal line on.
  elevatorServo.write(degrees);
  if (sleep == 1) {
#ifdef LOW_POWER
    LowPower.sleep(300);
#endif
#ifndef LOW_POWER
    delay(300);
#endif
  } else {
    delay(300);
  }
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(ELEVATOR_FET, LOW); // servo.detach() saves ~75 mA per servo. MOSFET saves an  additional ~4 mA per servo.
}

void readVoltage() {
  int rawVolt = analogRead(BAT_VOLTAGE_PIN);
  rawVolt = rawVolt * 2;
  data.volts = rawVolt * (3.3 / 1023.0) * 100;
}

void waitForFix() {
  data.sats = 0;
  data.fixType = 0;
  while ((data.fixType < 3) && (data.sats < 5)) { // Make sure location is valid before continuing.
    if (gps.getPVT()) {
      getGPSData();
#ifdef DEVMODE
      SerialUSB.println("Waiting for fix...");
      displayData();
#endif
    } else {
#ifdef DEVMODE
      SerialUSB.println("No PVT data received. Retrying...");
#endif
    }
  }
}

void I2CScan() {
  int nDevices = 0;

  SerialUSB.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      SerialUSB.print("I2C device found at address 0x");
      if (address < 16) {
        SerialUSB.print("0");
      }
      SerialUSB.print(address, HEX);
      SerialUSB.println("!");

      ++nDevices;
    } else if (error == 4) {
      SerialUSB.print("Unknown error at address 0x");
      if (address < 16) {
        SerialUSB.print("0");
      }
      SerialUSB.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    SerialUSB.println("No I2C devices found\n");
  } else {
    SerialUSB.println("Done\n");
  }
}

void streamData() {
  int available = Serial1.available();
  SerialUSB.print(available);
  SerialUSB.print(" available bytes ");
  while (Serial1.available() > 0) {
    SerialUSB.print(Serial1.read());
  }
}

void getIMUData() {
  while (Serial1.available() > 0) { // Clear the buffer.
    byte t = Serial1.read();
    SerialUSB.println(t);
  }
  digitalWrite(WRITE_PIN, HIGH);
#ifdef LOW_POWER
  LowPower.sleep(5);
#endif
#ifndef LOW_POWER
  delay(5);
#endif
  digitalWrite(WRITE_PIN, LOW);
  // 20 bytes equal 160 bits. 1 stop + start bit for every 8 bits, so 40 stop and start bits equal 200 bits. 9600/200 =~ 20 ms. Add 10 ms for safety.

#ifdef LOW_POWER
  LowPower.sleep(100);
#endif
#ifndef LOW_POWER
  delay(100);
#endif

  // if (Serial1.available() == sizeof(receivedData)) {
  Serial1.readBytes((byte *)&receivedData, sizeof(receivedData));
  data.yaw = receivedData.yaw;
  data.pitch = receivedData.pitch;
  data.roll = receivedData.roll;
  data.temp = receivedData.temp / 100;         // In Celsius.
  data.humidity = receivedData.humidity / 100; // In relative humidity.
  data.pressure = receivedData.pressure / 100; // In hPa.
  // } else {
  while (Serial1.available() > 0) { // Clear the buffer.
    byte t = Serial1.read();
    SerialUSB.println(t);
  }
}
// }

void calculate() {
  data.turnAngle = turningAngle(data.lat, data.lon, data.yaw, targetLat, targetLon);

  data.distanceMeters = calculateDistance(data.lat, data.lon, targetLat, targetLon); // Find the distance between the current location and the target.

  data.servoPositionElevator = pidMagicElevator(); // Change PID values in "settings.h" if you want.
  data.servoPositionRudder = pidMagicRudder();     // Change PID values in "settings.h" if you want.

  readVoltage();
}

void getGPSData() {
  data.lat = gps.getLatitude();
  data.lat = data.lat / 10000000;
  data.lon = gps.getLongitude();
  data.lon = data.lon / 10000000;
  data.alt = gps.getAltitude();
  data.alt = data.alt / 1000; // Convert to meters, I think?
  data.sats = gps.getSIV();
  data.fixType = gps.getFixType();
  data.speed = gps.getGroundSpeed();
  data.seconds = gps.getSecond();
  data.minutes = gps.getMinute();
  data.hours = gps.getHour();
  data.day = gps.getDay();
  data.month = gps.getDay();
  data.year = gps.getYear();
}

void displayData() {
  SerialUSB.print("Lat: ");
  SerialUSB.print(data.lat, 9);
  SerialUSB.print(" Lon: ");
  SerialUSB.print(data.lon, 9);
  SerialUSB.print(" Alt: ");
  SerialUSB.print(data.alt);
  SerialUSB.print(" Sats: ");
  SerialUSB.print(data.sats);
  SerialUSB.print(" Fix Type: ");
  SerialUSB.print(data.fixType);
  SerialUSB.print(" Speed: ");
  SerialUSB.print(data.speed);
  SerialUSB.print(" Date/Time: ");
  SerialUSB.print(data.year);
  SerialUSB.print("-");
  SerialUSB.print(data.month);
  SerialUSB.print("-");
  SerialUSB.print(data.day);
  SerialUSB.print(" ");
  SerialUSB.print(data.hours);
  SerialUSB.print(":");
  SerialUSB.print(data.minutes);
  SerialUSB.print(":");
  SerialUSB.print(data.seconds);
  SerialUSB.print(" Yaw: ");
  SerialUSB.print(data.yaw);
  SerialUSB.print(" Pitch: ");
  SerialUSB.print(data.pitch);
  SerialUSB.print(" Roll: ");
  SerialUSB.print(data.roll);
  SerialUSB.print(" Temp: ");
  SerialUSB.print(data.temp);
  SerialUSB.print(" Pressure: ");
  SerialUSB.print(data.pressure);
  SerialUSB.print(" Humidity: ");
  SerialUSB.print(data.humidity);
  SerialUSB.print(" Elevator Pos: ");
  SerialUSB.print(data.servoPositionElevator);
  SerialUSB.print(" Rudder Pos: ");
  SerialUSB.print(data.servoPositionRudder);
  SerialUSB.print(" Turn Angle: ");
  SerialUSB.print(data.turnAngle);
  SerialUSB.print(" Distance to Target: ");
  SerialUSB.print(data.distanceMeters);
  SerialUSB.print(" Voltage: ");
  SerialUSB.println(data.volts);
}

void onWake() {
  // Do nothing, as this will never happen.
}

void gpsWakeup() {
  digitalWrite(WAKEUP_PIN, LOW);
#ifdef LOW_POWER
  LowPower.sleep(1000);
#endif
#ifndef LOW_POWER
  delay(1000);
#endif
  digitalWrite(WAKEUP_PIN, HIGH);
#ifdef LOW_POWER
  LowPower.sleep(1000);
#endif
#ifndef LOW_POWER
  delay(1000);
#endif
  digitalWrite(WAKEUP_PIN, LOW);
}

void gpsConfig() {
  gps.factoryDefault();                                                        // Clear any saved configuration.
  if (gps.setDynamicModel(DYN_MODEL_AIRBORNE1g, VAL_LAYER_RAM_BBR) == false) { // Set the dynamic model to airborne mode with one g of thrust allowance.
#ifdef DEVMODE
    SerialUSB.println(F("*** Warning: setDynamicModel failed ***"));
#endif
  } else {
#ifdef DEVMODE
    SerialUSB.println(F("Dynamic platform model changed successfully!"));
#endif
  }
}
