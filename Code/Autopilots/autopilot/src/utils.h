/*
utils.h, part of StratoSoar MK2, for an autonomous glider.
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

// This is to predefine the functions to allow for default parameters. 

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