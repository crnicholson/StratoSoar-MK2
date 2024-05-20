/*
sendData.ino, part of StratoSoar MK2, for an autonomous glider.
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

// Use this code to test the transmission of serial data from the ATMega

// https://github.com/crnicholson/StratoSoar-MK2/.

#include <NeoSWSerial.h>

#define RX 10 
#define TX 9  
#define LED 13

#define BAUD_RATE 9600

NeoSWSerial mcuConn(RX, TX);

void setup() {
  mcuConn.begin(BAUD_RATE);
  pinMode(LED, OUTPUT);
}

void loop() {
  mcuConn.println("Testing serial communication.");
  digitalWrite(LED, HIGH);
  delay(10);
  digitalWrite(LED, LOW);
  delay(1000); 
}
