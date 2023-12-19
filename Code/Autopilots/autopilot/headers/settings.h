// This is a file that has all the values and variables for the main sketch
// This file is INTENDED to be EDITED
// Change these values to match your needs

// I2C adresses:
#define eepromI2CAddress 0x50

// Pins:
#define rudderPin 6
#define elevatorPin 5
#define parachutePin 4
#define rudderFET 8
#define elevatorFET 7
#define rudderBJT 2
#define elevatorBJT 3

// MCU serial pins:
#define mcuRX 9
#define mcuTX 10

// Target destination:
double targetLat = 42.36011635715412, targetLon = -71.09414503280355; // MIT coordinates (DON'T change this to a #define!)

// Testing coordinates:
double testLat = 42.3165665, testLon = -71.33451619; // NEST coordinates

// Use the servos?:
bool needParachute = false; // No need for parachutes right now
bool needElevator = true;
bool needRudder = true;

// Baud rates:
#define GPSBaud 9600
#define SerialBaud 9600
#define MCUBaud 9600

// EEPROM:
#define maxAddress 5000 // This is how many times you want to write to the EEPROM

// Servo names:
ServoTimer2_8mhz rudderServo; // Servo object
ServoTimer2_8mhz parachute;
ServoTimer2_8mhz elevatorServo;

// Desired pitch angle (in degrees):
#define setpointElevator 10

// Constants for PID control:
#define KpRudder 1.0   // Proportional gain
#define KiRudder 0.0   // Integral gain
#define KdRudder 0.0   // Derivative gain

#define KpElevator 1.0   // Proportional gain
#define KiElevator 0.0   // Integral gain
#define KdElevator 0.0   // Derivative gain

// These are the free parameters in the Mahony filter and fusion scheme,
// KpMahony for proportional feedback, KiMahony for integral with MPU-9250, angles start oscillating 
// at KpMahony=40. KiMahony does not seem to help and is not required. Change the values if you want:
#define KpMahony 30.0
#define KiMahony 0.0

// ^^^^ VERY IMPORTANT gyro/accel/magnetometer fields to fill out: ^^^^

// Accel offsets and correction matrix:
float M_B[3] {  40.667,  35.5346,  -67.9618}; 
float M_Ainv[3][3]
  {{  1.311423,  0.02533,  -0.027355},
  {  0.02433,  1.4148, 0.0424},
  {  -0.0275355, 0.0424,  1.359566}};
  
// Mag offsets and correction matrix:
float A_B[3] {   -284.23959,   210.444,  21.0823};
float A_Ainv[3][3]
  {{  0.59305,  0.006955,  0.006963},
  {  0.0060955,  0.614887,  -0.003107},
  {  0.006883,  -0.003147,  0.59344}};
  
float G_off[3] = {168.5, 59.6, 107.8}; // Raw offsets, determined for gyro at rest
