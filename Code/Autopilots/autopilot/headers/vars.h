// This is a file with a bunch variables that plays in with the main .ino sketch
// You can play with these values and names at your own risk, but catastrophic failure is likely

#define pi 3.14159265358979323846

// Vars:
int temp, pressure, yaw, pitch, speedMPH, servoPositionRudderDegrees, servoPositionElevatorDegrees; // add static here if there is a issue
double currentLat, currentLon, turnAngle;
bool spiral = false;
bool landed = false;
bool runEEPROM = true;
int eepromAddress = 0; // Current EEPROM address

// PID rudder stuff:
// Setpoint and input variables:
double setpointRudder = 0;   // Desired turn angle (in degrees) // this is just a random value for now, the code will change it
double inputRudder = 0.0;      // Current measured turn angle, also gibberish

// Variables for PID control:
double errorRudder = 0.0;      // Error (difference between setpoint and input)
double prevErrorRudder = 0.0;  // Previous error
double integralRudder = 0.0;   // Integral of the error

// PID elevator stuff:
// Input variables:
double inputElevator = 0.0;   // Current measured pitch angle, this is gibberish right now, and it will be changed by code

// Variables for PID control:
double errorElevator = 0.0;      // Error (difference between setpoint and input)
double prevErrorElevator= 0.0;   // Previous error
double integralElevator = 0.0;   // Integral of the error
