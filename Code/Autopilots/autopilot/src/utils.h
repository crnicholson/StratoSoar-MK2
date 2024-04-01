#pragma once

double toRadians(double);
double toDegrees(double);
double azimuth(double, double, double, double);
double turningAngle(double, double, double, double, double);
double calculateDistance(double, double, double, double);
int pidMagicElevator();
int pidMagicRudder();
void longPulse(int, int);
void shortPulse(int);
void moveRudder(int, int = 1);
void moveElevator(int, int = 1);
void readVoltage();
void I2CScan();
void streamData();
void getIMUData();
void calculate();
void getGPSData();
void displayData();
void waitForFix();
void onWake();
void gpsWakeup();
void gpsConfig();