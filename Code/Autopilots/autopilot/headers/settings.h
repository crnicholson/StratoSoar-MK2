#pragma once

// Change this file to match your needs.

// Enables.
#define DEVMODE // Toggle the serial monitor on and off.
// #define TEST_COORD // Use testing coordinates.
// #define CHANGE_TARGET // If the target location is too far away, change it to some place closer.
// #define SPIN_STOP  // Land the glider by sending it into a spin.
// #define STALL_STOP // Land the glider by repeatedly stalling it.
// #define NEED_PARACHUTE // Enable the parachute servo.
#define NEED_ELEVATOR // Enable the elevator servo.
#define NEED_RUDDER   // Enable the rudder servo.
#define DIVE_STALL    // Dive down when speed gets too low.
#define USE_EEPROM    // Toggle the usage of the EEPROM.

// Other settings.
#define SETPOINT_ELEVATOR 10 // Desired pitch angle (in degrees).
#define TOO_SLOW 5           // If DIVE_STALL is defined, and the MPH is equal to or below this threshold, dive down.
#define SLEEP_TIME 10000     // MS for sleep.

// Pins.
#define RUDDER_PIN 6
#define ELEVATOR_PIN 5
#define PARACHUTE_PIN 4
#define RUDDER_FET 8
#define ELEVATOR_FET 7
#define RUDDER_BJT 2
#define ELEVATOR_BJT 3
#define LED 13
#define WAKEUP_PIN 9

// Target destination.
double targetLat = 42.36011635715412, targetLon = -71.09414503280355; // MIT coordinates.

// Testing coordinates.
double testLat = 42.3165665, testLon = -71.33451619; // NEST coordinates.

// Baud rates.
#define SERIAL_BAUD_RATE 9600
#define BAUD_RATE 9600

// EEPROM.
#define MAX_ADDRESS 5000        // This is how many times you want to write to the EEPROM.
#define EEPROM_I2C_ADDRESS 0x50 // I2C address of the EEPROM.

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