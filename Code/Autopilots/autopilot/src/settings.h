/*
settings.h, part of StratoSoar MK2, for an autonomous glider.
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

#pragma once

// Change this file to match your needs.

// Enables.
// #define DEVMODE // Toggle the serial monitor on and off.
// #define LOW_POWER  // Activates low power mode. GPS will be lower power among others. Note that the serial monitor will no longer work.
// #define TEST_COORD // Use testing coordinates.
// #define CHANGE_TARGET // If the target location is too far away, change it to some place closer.
#define SPIN_STOP // Land the glider by sending it into a spin.
// #define STALL_STOP // Experimental, may be unsupported. Do not use. Land the glider by repeatedly stalling it.
// #define NEED_PARACHUTE // Enable the parachute servo.
// #define NEED_ELEVATOR // Enable the elevator servo.
#define NEED_RUDDER // Enable the rudder servo.
// #define DIVE_STALL    // Dive down when speed gets too low.
#define USE_EEPROM // Toggle the usage of the EEPROM.
// #define WIRELESS   // Experimental. If an HCO5 module is attached, a wireless serial connection can be made. More information in the docs.
// #define EEPROM_BUTTON // If enabled, the EEPROM will run for a configurable number of cycles after being pressed by a button. It will also record more data.
// #define GROUND // If doing ground testing, use this to enable faster updates.
// #define ERASE_EEPROM  // If enabled, the EEPROM will be erased before every flight. This is not recommended.
// #define SERVO_NONBLOCKING // Experimental, do not use.
// #define DROP_START // Starts the operations when a decreasing altitude is detected.

// Time settings.
#define FAST_UPDATE_PERIOD 3600   // In this time, the glider will update its yaw more frequently. The time is in seconds.
#define GPS_SLEEP 10000           // This is how long the glider will wait until it will get a new GPS fix, saving power in between fixes. The time is in milliseconds and does not apply if ground is enabled.
#define GPS_GROUND_SLEEP 10000    // This is how long the glider will wait until it will get a new GPS fix, saving power in between fixes. The time is in milliseconds and does not apply if ground is disabled.
#define SPIRAL_SLEEP 500          // If spiraling down, how long should the glider sleep for in between GPS and parachute checkups?
#define BELOW_THRESHOLD_SLEEP 500 // If the yaw is below the threshold, the glider will sleep for this many seconds before checking if the drift is enough.
#define ABOVE_THRESHOLD_SLEEP 500 // If the yaw is above the threshold, the glider will sleep for this long. Note that 200 will be subtracted from this because there was a 200 ms delay in between moving servos.
#define ABV_THRS_FST_UPDT_SLP 500 // (Above threshold fast update period sleep). If the yaw is above the threshold and if the glider is in the fast update period of flight, the glider will for this long. Same note as above, too.
#define FLIGHT_TIME 240           // In minutes, the time expected to be in flight for, used for calculating the time between EEPROM writes.

// Other settings.
#define SETPOINT_ELEVATOR 10   // Desired pitch angle (in degrees).
#define TOO_SLOW 5             // If DIVE_STALL is defined, and the MPH is equal to or below this threshold, dive down.
#define YAW_DFR_THRESHOLD 5    // The threshold needed to move the servos.
#define ARM_ALT 10             // Meters before the altitude is considered safe to arm the glider if DROP_START is enabled.
#define SPIN_DEGREE 180        // The angle of the servo when spiraling down to the ground. 90 degrees is the middle.
#define REF_PRESSURE 101659.39 // Daily pressure at sea level today in Pa.

// Threshold for parachute and spiraling.
#define PARACHUTE_ALT_THRESHOLD 500 // If the glider is under this, and the distance is less than PARACHUTE_DST_THRESHOLD, open the parachute.
#define PARACHUTE_DST_THRESHOLD 100 // If the glider is under PARACHUTE_ALT_THRESHOLD, and the distance is less than this, open the parachute.
#define SPIRAL_ALT_THRESHOLD 0      // If the glider is above this, and the distance is less than SPIRAL_DST_THRESHOLD, spiral the glider down.
#define SPIRAL_DST_THRESHOLD 30     // If the glider is above SPIRAL_ALT_THRESHOLD, and the distance is less than this, spiral the glider down.

// Pins. NOTE do not change these if you are using the PCB.
// (checked with breadboard, code, and schematic on Feb 1st, 2024. Last change to pins was before Feb 1st.)
// SPI is available (unused) on hardware pins 19-22. This will be on the GPIO header of the autopilot.
// I2C is used on hardware pins 31 and 32. Also on GPIO header of autopilot.
// Serial1 is used on 15 and 16 (Arduino pins 1 and 0 respectively).
#define RUDDER_PIN 2        // Hardware pin 23. Servo signal pin that attaches to "input" of BJT.
#define ELEVATOR_PIN 3      // Hardware pin 14. Servo signal pin that attaches to "input" of BJT.
#define PARACHUTE_PIN 4     // Hardware pin 13. Servo signal pin that attaches to "input" of BJT.
#define RUDDER_FET 5        // Hardware pin 24. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define ELEVATOR_FET 6      // Hardware pin 29. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define PARACHUTE_FET 7     // Hardware pin 30. Pin that attaches to switching part of FET. This allows the servo to be turned on.
#define RUDDER_BJT 8        // Hardware pin 11. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define PARACHUTE_BJT 9     // Hardware pin 12. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define ELEVATOR_BJT 10     // Hardware pin 27. Pin that attaches to switching part of BJT. This allows the signal to pass to the servo.
#define WAKEUP_PIN 11       // Hardware pin 25. Pin that attaches to the EXTINT of ublox module.
#define FALSE_WAKEUP_PIN 12 // Hardware pin 28. Pin that is used for the interrupt that puts the glider into deep sleep. Do not connect anything.
#define LED 13              // Hardware pin 26. Pin that is connected to the main green LED.
#define ERR_LED A0          // Hardware pin 3. Pin that connects ot the red error LED.
#define BAT_VOLTAGE_PIN A1  // Hardware pin 7. Pin for battery voltage measurement.
#define WRITE_PIN A2        // Hardware pin 8. Pin that is driven high to receive serial data from the ATMega.
#define BUTTON A3           // Hardware pin 9. Used for enabling EEPROM.
#define WIRELESS_RX A4      // Hardware pin 10. Pin that connects to the HCO5. More info in the docs.
#define WIRELESS_TX A5      // Hardware pin 47. Pin that connects to the HCO5. More info in the docs.

// Target destination.
double targetLat = 42.263551, targetLon = -71.320015; // Glenwood Cemetery.
// double targetLat = 42.315373, targetLon = -71.334379; // Landscaping place near NEST.
// double targetLat = 42.31619, targetLon = -71.36740; // House 9 on Pemberton Road.
// double targetLat = 42.3408625, targetLon = -71.3780515;
// double targetLat = 42.3421048, targetLon = -71.3778945;
// double targetLat = 42.31549, targetLon = -71.33391; // Blue tarp at NEST.
// double targetLat = 42.31670, targetLon = -71.36622; // House on Stanton Street.

// Testing glider coordinates.
// double testLat = 42.31610, testLon = -71.33468; // NEST coordinates.
// double testLat = 42.31746, testLon = -71.36467; // Local bank coordinates.
// double testLat = 42.3401859, testLon = -71.3781675;
// double testLat = 42.3390271, testLon = -71.3782380;
double testLat = 42.31622, testLon = -71.33370; // Woods at NEST.

// Backup coordinates, furthest.
#define FURTHEST_LAT 42
#define FURTHEST_LON -71
#define FURTHEST_ALT_THRS 0
#define FURTHEST_DST_THRS 1000 // If the glider is below FURTHEST_ALT_THRS and the distance is greater to the target than this, relocate the target to FURTHEST_LAT and FURTHEST_LON.

// Baud rates.
#define SERIAL_BAUD_RATE 115200 // Serial monitor baud rate.
#define BAUD_RATE 9600          // Serial link to the ATMega.

// EEPROM settings.
#define EEPROM_CYCLES 10   // This is the number of times the EEPROM will record data after the button is pressed.
#define BYTES_PER_CYCLE 16 // The number of bytes written to the EEPROM per cycle.
#define EEPROM_BUFFER .25  // Extra added time when calculating the time between EEPROM cycles.
// As of 5/23/24: With EEPROM_BUTTON, BYTES_PER_CYCLE is 16. Without EEPROM_BUTTON, BYTES_PER_CYCLE is 6.

// Servo objects and names.
#ifdef NEED_RUDDER
Servo rudderServo;
#endif
#ifdef NEED_PARACHUTE
Servo parachute;
#endif
#ifdef NEED_ELEVATOR
Servo elevatorServo;
#endif

// Constants for PID control.
#define KP_RUDDER 1.0 // Proportional gain.
#define KI_RUDDER 0.0 // Integral gain.
#define KD_RUDDER 0.0 // Derivative gain.

#define KP_ELEVATOR 1.0 // Proportional gain.
#define KI_ELEVATOR 0.0 // Integral gain.
#define KD_ELEVATOR 0.0 // Derivative gain.
