#pragma once

// Change this file to match your needs.

// Enables.
#define DEVMODE    // Toggle the serial monitor on and off.
#define TEST_COORD // Use testing coordinates.
// #define CHANGE_TARGET // If the target location is too far away, change it to some place closer.
// #define SPIN_STOP  // Land the glider by sending it into a spin.
// #define STALL_STOP // Land the glider by repeatedly stalling it.
// #define NEED_PARACHUTE // Enable the parachute servo.
#define NEED_ELEVATOR // Enable the elevator servo.
#define NEED_RUDDER   // Enable the rudder servo.
// #define DIVE_STALL    // Dive down when speed gets too low.
// #define USE_EEPROM // Toggle the usage of the EEPROM.
// #define WIRELESS   // If an HCO5 module is attached, a wireless serial connection can be made. More information in the docs.

// Sleep time settings.
#define SPIRAL_SLEEP 500 // If spiraling down, how long should the glider sleep for in between GPS and parachute checkups?
#define 

// Other settings.
#define SETPOINT_ELEVATOR 10 // Desired pitch angle (in degrees).
#define TOO_SLOW 5           // If DIVE_STALL is defined, and the MPH is equal to or below this threshold, dive down.
#define YAW_DFR_THRESHOLD -5 // The threshold needed to move the servos. Make sure it is a negative number.

// Threshold for parachute and spiraling.
#define PARACHUTE_ALT_THRESHOLD 500 // If the glider is under this, and the distance is less than PARACHUTE_DST_THRESHOLD, open the parachute.
#define PARACHUTE_DST_THRESHOLD 100 // If the glider is under PARACHUTE_ALT_THRESHOLD, and the distance is less than this, open the parachute.
#define SPIRAL_ALT_THRESHOLD 600    // If the glider is above this, and the distance is less than SPIRAL_DST_THRESHOLD, spiral the glider down.
#define SPIRAL_DST_THRESHOLD 100    // If the glider is above SPIRAL_ALT_THRESHOLD, and the distance is less than this, spiral the glider down.

// Pins (checked with breadboard, code, and schematic on Feb 1st, 2024. Last change to pins was before Feb 1st.)
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
// #define GPIO A3 // Hardware pin 9. Unused GPIO on autopilot.
#define WIRELESS_RX A4 // Hardware pin 10. Pin that connects to the HCO5. More info in the docs.
#define WIRELESS_TX A5 // Hardware pin 47. Pin that connects to the HCO5. More info in the docs.

// Target destination.
double targetLat = 42.36011635715412, targetLon = -71.09414503280355; // MIT coordinates.

// Testing coordinates.
double testLat = 42.3165665, testLon = -71.33451619; // NEST coordinates.

// Baud rates.
#define SERIAL_BAUD_RATE 115200 // Serial monitor baud rate.
#define BAUD_RATE 9600          // Serial link to the ATMega.

// EEPROM.
#define MAX_ADDRESS 64000       // This is how many 8 bit pages your EEPROM has, 64,000 for a 512 kilobit EEPROM.
#define EEPROM_I2C_ADDRESS 0x50 // I2C address of the EEPROM.
#define WRITE_TIME 1.5          // The seconds between EEPROM writings as calculated previously.

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