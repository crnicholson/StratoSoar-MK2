#define RX 10                 // TX pin of the mcuConn.
#define TX 11                 // RX pin of the mcuConn.
#define SEND_MS 6000          // Send data every "SEND_MS" milliseconds.
#define BAUD_RATE 9600        // Baud rate for the mcuConn.
#define SERIAL_BAUD_RATE 9600 // Baud rate for the serial monitor.
#define Kp 30.0               // These are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral.
#define Ki 0.0                // With MPU-9250, angles start oscillating at Kp = 0. Ki does not seem to help and is not required.
#define MAG_DEC 14.5          // Conventional nav, yaw increases CW from North, corrected for local magnetic declination. Find yours here: http://www.ngdc.noaa.gov/geomag-web/#declination.
// #define DEVMODE            // Comment out to use the serial monitor.

// Accel offsets and correction matrix.
float A_B[3]{539.75, 218.36, 834.53};
float A_Ainv[3][3]{
    {0.51280, 0.00230, 0.00202},
    {0.00230, 0.51348, -0.00126},
    {0.00202, -0.00126, 0.50368}};

// Mag offsets and correction matrix.
float M_B[3]{18.15, 28.05, -36.09};
float M_Ainv[3][3]{
    {0.68093, 0.00084, 0.00923},
    {0.00084, 0.69281, 0.00103},
    {0.00923, 0.00103, 0.64073}};

float G_off[3] = {-299.7, 113.2, 202.4}; // Raw offsets, determined for gyro at rest.
