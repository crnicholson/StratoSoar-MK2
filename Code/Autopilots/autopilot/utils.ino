#include "headers/settings.h"
#include <Arduino.h>

double deg2rad(double deg) {
  return deg * pi / 180.0;
}

double toDegrees(double rad) {
  return rad * 180.0 / pi;
}

double turningAngle(double currentLat, double currentLon, double heading1, double targetLat, double targetLon) {
  double a = azimuth(currentLat, currentLon, targetLat, targetLon);
  double h = toRadians(heading1);
  double angle = a - h;
  if (angle < -pi)
    angle += 2 * pi;
  if (angle > pi)
    angle -= 2 * pi;
  return toDegrees(angle);
}

double azimuth(double currentLat, double currentLon, double targetLat, double targetLon) {
  double dLon = deg2rad(targetLon - currentLon);
  double dPhi = log(tan(deg2rad(targetLat) / 2 + pi / 4) / tan(deg2rad(currentLat) / 2 + pi / 4));
  if (fabs(dLon) > pi) {
    dLon = dLon > 0 ? -(2 * pi - dLon) : (2 * pi + dLon);
  }
  return fmod((atan2(dLon, dPhi) + 2 * pi), (2 * pi));
}

// Haversine formula to calculate distance between two coordinates.
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(toRadians(lat1)) * cos(toRadians(lat2)) *
                 sin(dLon / 2) * sin(dLon / 2);
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
  if (Serial1.available() >= 6) {     // Check to see how many bytes we have to read
    byte yawReceive = Serial1.read(); // Read the transmitted bytes from autopilotIMU.vx.x
    byte pitchReceive = Serial1.read();
    byte negativePitch = Serial1.read();
    byte tempReceive = Serial1.read();
    byte negativeTemp = Serial1.read();
    byte pressureReceive = Serial1.read();

    yaw = int(yawReceive) * 2;             // Convert the received byte back to an integer
    pitch = int(pitchReceive);             // Convert the received byte back to an integer
    temp = int(tempReceive);               // Convert the received byte back to an integer
    pressure = int(pressureReceive) * 500; // Convert the received byte back to an integer

    if (negativePitch == 1) { // Making some things negative if needed
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