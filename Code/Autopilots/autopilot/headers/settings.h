#pragma once

// Change this file to match your needs.

// Enables.
#define DEVMODE // Toggle the serial monitor on and off.
// #define TEST_COORD // Use testing coordinates.
// #define CHANGE_TARGET // If the target location is too far away, change it to some place closer.
// #define SPIN_STOP  // Land the glider by sending it into a spin.
// #define STALL_STOP // Land the glider by repeatedly stalling it.

// Pins.
#define RUDDER_PIN 6
#define ELEVATOR_PIN 5
#define PARACHUTE_PIN 4
#define RUDDER_FET 8
#define ELEVATOR_FET 7
#define RUDDER_BJT 2
#define ELEVATOR_BJT 3
#define LED 13

// Target destination.
double targetLat = 42.36011635715412, targetLon = -71.09414503280355; // MIT coordinates.

// Testing coordinates.
double testLat = 42.3165665, testLon = -71.33451619; // NEST coordinates.

// Toggle the usage of the servos.
bool needParachute = false;
bool needElevator = true;
bool needRudder = true;

// Baud rates.
#define SERIAL_BAUD_RATE 9600
#define BAUD_RATE 9600

// EEPROM.
#define MAX_ADDRESS 5000        // This is how many times you want to write to the EEPROM.
#define EEPROM_I2C_ADDRESS 0x50 // I2C address of the EEPROM.

// Servo names.
Servo rudderServo;
Servo parachute;
Servo elevatorServo;

// Desired pitch angle (in degrees).
#define SETPOINT_ELEVATOR 10

// Constants for PID control.
#define KP_RUDDER 1.0 // Proportional gain.
#define KI_RUDDER 0.0 // Integral gain.
#define KD_RUDDER 0.0 // Derivative gain.

#define KP_ELEVATOR 1.0 // Proportional gain.
#define KI_ELEVATOR 0.0 // Integral gain.
#define KD_ELEVATOR 0.0 // Derivative gain.