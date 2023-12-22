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
  double distance = R * c;
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
  errorElevator = SETPOINT_ELEVATOR - pitch; // Calculate the error.

  int outputElevator = KP_ELEVATOR * errorElevator + KI_ELEVATOR * integralElevator + KD_ELEVATOR * (errorElevator - prevErrorElevator); // Calculate the output.

  servoPositionElevatorDegrees = 90 - outputElevator; // Adjust servo position based on the output.

  // Update previous error and integral.
  prevErrorElevator = errorElevator;
  integralElevator += errorElevator;

  int servoPositionElevatorNew = map(servoPositionElevatorDegrees, 0, 180, 750, 2250);

  return servoPositionElevatorNew;
}

int pidMagicRudder() {
  setpointRudder = yaw + turnAngle;
  inputRudder = yaw;

  errorRudder = setpointRudder - yaw; // Calculate the error.

  int outputRudder = KP_RUDDER * errorRudder + KI_RUDDER * integralRudder + KD_RUDDER * (errorRudder - prevErrorRudder); // Calculate the output.

  servoPositionRudderDegrees = 90 + outputRudder; // Adjust servo position based on the output.

  // Update previous error and integral.
  prevErrorRudder = errorRudder;
  integralRudder += errorRudder;

  int servoPositionRudderNew = map(servoPositionRudderDegrees, 0, 180, 1250, 1750);

  return servoPositionRudderNew;
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

void receiveData() {
  if (Serial1.available() >= 6) {     // Check to see how many bytes we have to read.
    byte yawReceive = Serial1.read(); // Read the transmitted bytes from autopilotIMU.
    byte pitchReceive = Serial1.read();
    byte negativePitch = Serial1.read();
    byte tempReceive = Serial1.read();
    byte negativeTemp = Serial1.read();
    byte pressureReceive = Serial1.read();

    yaw = int(yawReceive) * 2;             // Convert the received byte back to an integer.
    pitch = int(pitchReceive);             // Convert the received byte back to an integer.
    temp = int(tempReceive);               // Convert the received byte back to an integer.
    pressure = int(pressureReceive) * 500; // Convert the received byte back to an integer.

    if (negativePitch == 1) { // Making some things negative if needed.
      pitch = pitch * -1;
    }

    if (negativeTemp == 1) {
      temp = temp * -1;
    }
  }
}

void moveRudder(degrees) {
  digitalWrite(RUDDER_FET, HIGH); // Turn servo on.
  digitalWrite(RUDDER_BJT, LOW);  // Turn signal line on.
  rudderServo.write(degrees);
  delay(200);
  digitalWrite(RUDDER_BJT, HIGH);
  digitalWrite(RUDDER_FET, LOW);
}

void moveElevator(degrees) {
  digitalWrite(ELEVATOR_FET, HIGH); // Turn servo on.
  digitalWrite(ELEVATOR_BJT, LOW);  // Turn signal line on.
  elevatorServo.write(degrees);
  delay(200);
  digitalWrite(ELEVATOR_BJT, HIGH);
  digitalWrite(ELEVATOR_FET, LOW); // servo.detach() saves ~75 mA per servo. MOSFET saves additional ~4 mA per servo.
}