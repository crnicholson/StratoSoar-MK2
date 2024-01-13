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
  double h = toRadians(h);
  double angle = a - head;
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

void writeToEEPROM(byte EEPROMAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(EEPROMAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
  delay(5);
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

void longPulse() {
  digitalWrite(LED, HIGH);
  delay(250);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(250);
  digitalWrite(LED, LOW);
}

void shortPulse() {
  digitalWrite(LED, HIGH);
  delay(250);
  digitalWrite(LED, LOW);
}

void moveRudder(int degrees) {
  digitalWrite(RUDDER_FET, HIGH); // Turn servo on.
  digitalWrite(RUDDER_BJT, LOW);  // Turn signal line on.
  rudderServo.write(degrees);
  LowPower.deepSleep(200);
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
}

void moveElevator(int degrees) {
  digitalWrite(ELEVATOR_FET, HIGH); // Turn servo on.
  digitalWrite(ELEVATOR_BJT, LOW);  // Turn signal line on.
  elevatorServo.write(degrees);
  LowPower.deepSleep(200);
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(ELEVATOR_FET, LOW); // servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
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

void getIMUData() {
  if (Serial1.available() >= sizeof(receivedData)) {
    Serial1.readBytes((byte *)&receivedData, sizeof(receivedData));
    data.yaw = int(receivedData.yaw);
    data.pitch = int(receivedData.pitch);
    data.roll = int(receivedData.roll);
    data.temp = int(receivedData.temp);
    data.humidity = int(receivedData.humidity);
    data.pressure = int(receivedData.pressure);
  }
  /*
  if (Serial1.available() >= 6) {     // Check to see how many bytes we have to read.
    byte yawReceive = Serial1.read(); // Read the transmitted bytes from autopilotIMU (the ATMega).
    byte pitchReceive = Serial1.read();
    byte negativePitch = Serial1.read();
    byte tempReceive = Serial1.read();
    byte negativeTemp = Serial1.read();
    byte pressureReceive = Serial1.read();

    data.yaw = int(yawReceive) * 2;             // Convert the received byte back to an integer.
    data.pitch = int(pitchReceive);             // Convert the received byte back to an integer.
    data.temp = int(tempReceive);               // Convert the received byte back to an integer.
    data.pressure = int(pressureReceive) * 500; // Convert the received byte back to an integer.

    if (negativePitch == 1) { // Making some things negative if needed.
      data.pitch = data.pitch * -1;
    }

    if (negativeTemp == 1) {
      data.temp = data.temp * -1;
    }
  }
  */
}

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
  delay(1000);
  digitalWrite(WAKEUP_PIN, HIGH);
  delay(1000);
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