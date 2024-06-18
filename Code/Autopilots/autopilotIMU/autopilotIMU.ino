// WARNING:
// This is still a work in progress! Code may not work!

/*
autopilotIMU.ino, part of StratoSoar MK2, for an autonomous glider.
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

// https://github.com/crnicholson/StratoSoar-MK2/.

// Original work done by S.J. Remington 3/2020.
// Adapted to work in an autopilot system by Charles Nicholson.

// Use this code in conjuction with the autopilot sketch to obtain pitch, roll, and yaw values that can be used in the autopilot system.

// Please read this repo over before using this code: https://github.com/jremington/MPU-9250-AHRS.
// Also note that I take no credit for the AHRS, only the interfacing between the autopilot and this sketch.

// ***** Calibration *****
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work, and the gyro offset must be determined.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html.
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/.

// To collect data for calibration, use the programs MPU9250_cal and Magneto 1.2 from sailboatinstruments.blogspot.com.
// Magneto can be installed on all Window machines with a simple .exe file, and requires a .txt or .csv from MPU9250_cal.ino.
// The serial output from MPU9250_cal.ino can be copied and pasted into a spreadsheet, then downloaded and uploaded to Magneto.
// The output from Magneto cannot be copied and pasted or downloaded, so you have to manually input the values to M_B, M_Ainv,
// A_B, and A_Ainv. The values you input are in the format that is pretty much how you see them in Magneto.
// You also have to input the G_off array, which is not determined by Magneto, but by MPU9250_Cal in the beginning of the program.

// ***** To-Do *****
// Format code - add periods, capitalize, format above documentation
// Write documentation

#include "headers/settings.h" // Change settings here.
#include "src/I2Cdev.h"       // Credit: https://github.com/jremington/MPU-9250-AHRS
#include "src/MPU9250.h"      // Credit: https://github.com/jremington/MPU-9250-AHRS
#include <NeoSWSerial.h>
#include <TinyBME280.h>
#include <Wire.h>

unsigned long now = 0, last = 0; // Micros() timers.
float deltat = 0;                // Loop time in seconds.
long nowPrint, lastPrint = 0;    // For printing data.

static float q[4] = {1.0, 0.0, 0.0, 0.0}; // Vector to hold quaternion.

struct __attribute__((packed)) dataStruct {
  float pitch;
  float roll;
  float yaw;
  long temp;
  float pressure;
  long humidity;
} data;

struct __attribute__((packed)) testStruct {
  float pitch = 90.9;
  float roll = 63.2;
  float yaw = 45;
  long temp = 6123;
  // int32_t pressure;
  long humidity = 5613;
} test;

MPU9250 accelgyro;
I2Cdev I2C_M;
NeoSWSerial mcuConn(RX, TX);

char s[60]; // Snprintf buffer.
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float Mxyz[3];
#define gscale (250. / 32768.0) * (PI / 180.0) // Gyro default 250 LSB per d/s -> rad/s.

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(WRITE_PIN, INPUT);
  digitalWrite(LED, LOW);
  longPulse(LED);
  Wire.begin();
  // BME280setI2Caddress(BME_ADDRESS); // This gave me many issues in testing, don't do it!
  BME280setup();
  mcuConn.begin(BAUD_RATE);
  accelgyro.initialize(); // Initialize MPU9250.
#ifdef DEVMODE
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) // Wait for connection.
    ;
  Serial.println(accelgyro.testConnection() ? "MPU9250 is all good" : "MPU9250 is missing!"); // Verify connection.
#endif
  delay(1000);
#ifdef DEVMODE
  I2CScan();
#endif
  longPulse(LED);
  delay(5000);
}

void loop() {
  get_MPU_scaled();
  now = micros();
  deltat = (now - last) * 1.0e-6; // Seconds since last update.
  last = now;

  // Correct for differing accelerometer and magnetometer alignment by circularly permuting mag axes.
  MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[1], Mxyz[0], -Mxyz[2], deltat);

  // Standard orientation: X North, Y West, Z Up.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, they get the correct orientation.
  // The rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // Some good reading: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.

  // This will malfunction for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock.
  data.roll = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
  data.pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
  data.yaw = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));

  // To degrees.
  data.yaw *= 180.0 / PI;
  data.pitch *= 180.0 / PI;
  data.roll *= 180.0 / PI;

  data.yaw = -data.yaw + MAG_DEC;
  if (data.yaw < 0) {
    data.yaw += 360.0;
  }
  if (data.yaw > 360.0) {
    data.yaw -= 360.0;
  }

  if (digitalRead(WRITE_PIN)) {
    data.pressure = BME280pressure(); // Pressure in Pa.
    data.temp = BME280temperature();  // Temp in C.
    data.humidity = BME280humidity(); // Humidity in %RH.

    mcuConn.write((byte *)&data, sizeof(data));

    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
  }

#ifdef DEVMODE
  nowPrint = millis();
  if (nowPrint - lastPrint > 1000) {
    lastPrint = nowPrint;
    data.temp = BME280temperature();  // Temp in C.
    data.humidity = BME280humidity(); // Humidity in %RH.

    Serial.print(data.yaw);
    Serial.print(", ");
    Serial.print(data.pitch);
    Serial.print(", ");
    Serial.print(data.temp);
    Serial.print(", ");
    Serial.println(data.humidity);
  }
#endif
}

// Functions below:

void get_MPU_scaled(void) {
  float temp[3];
  int i;
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

  Gxyz[0] = ((float)gx - G_off[0]) * gscale; // 250 LSB(d/s) default to radians/s.
  Gxyz[1] = ((float)gy - G_off[1]) * gscale;
  Gxyz[2] = ((float)gz - G_off[2]) * gscale;

  Axyz[0] = (float)ax;
  Axyz[1] = (float)ay;
  Axyz[2] = (float)az;
  // Apply offsets (bias) and scale factors from Magneto.
  for (i = 0; i < 3; i++)
    temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  Mxyz[0] = (float)mx;
  Mxyz[1] = (float)my;
  Mxyz[2] = (float)mz;
  // Supply offsets and scale factors from Magneto.
  for (i = 0; i < 3; i++)
    temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

// Mahony scheme uses proportional and integral filtering on the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat) {
  // Vector to hold integral error for Mahony method.
  static float eInt[3] = {0.0, 0.0, 0.0};
  // Short name local variable for readability.
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic.
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  /*
    // Already done in loop()

    // Normalise accelerometer measurement.
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN.
    norm = 1.0f / norm;       // Use reciprocal for division.
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement.
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN.
    norm = 1.0f / norm;       // Use reciprocal for division.
    mx *= norm;
    my *= norm;
    mz *= norm;
  */
  // Reference direction of Earth's magnetic field.
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field.
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of the reference vectors.
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f) {
    eInt[0] += ex; // Accumulate integral error.
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback.
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }

  // Apply P feedback.
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;

  // Integrate rate of change of quaternion.
  // Small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447.
  gx = gx * (0.5 * deltat); // Pre-multiply common factors.
  gy = gy * (0.5 * deltat);
  gz = gz * (0.5 * deltat);
  float qa = q1;
  float qb = q2;
  float qc = q3;
  q1 += (-qb * gx - qc * gy - q4 * gz);
  q2 += (qa * gx + qc * gz - q4 * gy);
  q3 += (qa * gy - qb * gz + q4 * gx);
  q4 += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion.
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

void I2CScan() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("!");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done\n");
  }
}

void longPulse(int pin) {
  digitalWrite(pin, HIGH);
  delay(250);
  digitalWrite(pin, LOW);
  delay(500);
  digitalWrite(pin, HIGH);
  delay(250);
  digitalWrite(pin, LOW);
}

void shortPulse(int pin) {
  digitalWrite(pin, HIGH);
  delay(250);
  digitalWrite(pin, LOW);
}
