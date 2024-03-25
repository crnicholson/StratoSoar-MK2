// Program to collect raw data from MPU-9250 accelerometer and magnetometer for later correction.
// S. J. Remington 3/2020, adapted by Charles Nicholson, 2024.

// Use Magneto 1.2 to calculate corrections for both accelerometer and magnetometer.
// Get Magneto here: http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html.

// Usage:
//  1. Allow the sensor to sit still upon program startup for gyro bias data collection.
//  2. Turn the sensor very slowly and carefully to avoid excess acceleration.
//  3. When calibrating, make sure the top of the IMU touches all the insides of a sphere (figuratively).
//  4. Copy and paste the data from the serial monitor (just the numbers, no text) into a spreadsheet.
//  5. You may need to split the cells in the spreadsheet.
//  6. Download just the mag column and import it into Magneto.
//  7. Fill in the calibration array in "settings.h" for the mag. Fill it in the way you see it in Magneto.
//  8. Download just the accel column and import it into Magneto.
//  9. Fill in the calibration array in "settings.h" for the accel. Fill it in the way you see it in Magneto.
// 10. Enter the rms for the mag and the accel in "settings.h".
// 11. Enter raw gyro offsets into "settings.h".
// 12. Done!

#include "Wire.h"
#include "src/I2Cdev.h" // Arduino doesn't allow for relative paths, so you have to have two src folders.
#include "src/MPU9250.h"

// Class default I2C address is 0x68.
// AD0 low = 0x68 (default for InvenSense evaluation board).
// AD0 high = 0x69.
MPU9250 imu;
I2Cdev I2C_M;

#define sample_num 300 // Number of acc/mag points to collect.

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3] = {0};
float Gxyz[3] = {0};
float Mxyz[3] = {0};

// Don't do anything here!
// Previously determined scale and offsets for accel and mag.
float A_cal[6] = {0}; // 0..2 scale, 3..5 offsets.
float M_cal[6] = {0};

int N = sample_num;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("Initializing I2C devices...");
  imu.initialize();

  Serial.println("Testing device connections...");
  Serial.println(imu.testConnection() ? "MPU-9250 found." : "MPU-9250 not found.");
  Serial.println("Gyro bias collection - keep sensor still.");

  delay(2000);

  for (int i = 0; i < N; i++) {
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] += float(gx);
    Gxyz[1] += float(gy);
    Gxyz[2] += float(gz);
  }

  Serial.print("Done. Raw gyro offsets: ");
  Serial.print(Gxyz[0] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[1] / N, 1);
  Serial.print(", ");
  Serial.print(Gxyz[2] / N, 1);
  Serial.println();

  Serial.print("Collecting ");
  Serial.print(N);
  Serial.println(" points for scaling, 3/second.");
  Serial.println("Turn sensor very slowly in 3D space. Make sure to rotate and move all sides.");
  delay(2000);

  float M_mag = 0, A_mag = 0;
  int i, j;
  j = N;
  while (j-- >= 0) {
    getAcc_Mag_raw();
    for (i = 0; i < 3; i++) {
      M_mag += Mxyz[i] * Mxyz[i];
      A_mag += Axyz[i] * Axyz[i];
    }
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.print(az);
    Serial.print(" ");
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.println(mz);
    delay(300);
  }

  Serial.print("Done. ");
  Serial.print("rms Acc = ");
  Serial.print(sqrt(A_mag / N));
  Serial.print(", rms Mag = ");
  Serial.println(sqrt(M_mag / N));
}

void loop() {
}

void getAcc_Mag_raw() {
  imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (float)ax;
  Axyz[1] = (float)ay;
  Axyz[2] = (float)az;
  Mxyz[0] = (float)mx;
  Mxyz[1] = (float)my;
  Mxyz[2] = (float)mz;
}

void getGyro_scaled() {
  imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (float)gx * 250 / 32768; // 250 LSB(d/s)
  Gxyz[1] = (float)gy * 250 / 32768;
  Gxyz[2] = (float)gz * 250 / 32768;
}
