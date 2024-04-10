# StratoSoar Docs - A Work in Progress!

This is split into a few different sections to be more organized. These are as follows:

- General
- PCB
- Code and Programming
- Glider
- Usage

# General

## StratoSoar File Layout

## Power Consumption

# PCB

## Required Parts for the PCB

<!--


**NOTE:** Incomplete, more to come soon.



This codebase is meant to work with the StratoSoar PCB, also on this GitHub repository. On it are these basic components:



-  **Microcontroller** - [SAMD21G18A](https://www.microchip.com/en-us/product/atsamd21g18). This is a modern Cortex-M0 microcontroller by Microchip. It runs on ARM and has low-power capabilities.

-  **GPS Module** - [MAX-M10S](https://www.u-blox.com/en/product/max-m10-series). This is a top-of-the-line GPS module from u-blox.

-  **Environmental Sensor** - [BME280](https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/). This is a sensor from Bosch proven to work on HAB flights. Measures temperature, pressure, and humidity.

-  **Boost Converter** - [TPS61200](https://www.ti.com/product/TPS61200). This is a boost converter from Texas Instruments. This is used to convert from 1.5v (or 3v with two batteries) to 5v for the servos. The output of this is hooked up to the input of the LDO.

-  **LDO Step-Down Regulator** - [MCP1700](https://www.microchip.com/en-us/product/mcp1700). This is 5v to 3.3v low-dropout and low-quiescent current voltage regulator, supplying everything besides the servos.

-  **Internal Measurement Unit (IMU)** - [MPU-9250](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/). This little device is an accelormeter, gyroscope, and magnetometer all-in-one that can be used to find pitch, roll, and yaw among other metrics.



This list is not at all comprehensive, but it features the basic components. Check out the schematic for more!
-->

## PCB Assembly

## PCB Configuration

**NOTE:** This is only relevant for MK2.1 and upwards.

Jumpers:
- **JP1**: Solder to enable PPS LED of GPS module.
- **JP2**: Solder to enable one cell battery operation. Default is two cell battery operation. 
- **JP3**: Solder to enable pin 13 ATMega LED. 
- **JP4**: Solder to enable SAMD error LED.
- **JP5**: Solder to enable SAMD pin 13 LED. 
- **JP6 + JP7**: Solder to enable external 32.768 kHz crystal. Note that the crystal must be soldered and enabled if the TCXO is not placed. 

Switches:
- **SW2**: Switch towards the GPS module to disable EEPROM. Switch away to enable EEPROM.
- **SW3**: Switch towards the GPS module to supply power via the PCB-mounted batteries. 

## PCB Headers and Interfaces 

# Code and Programming

## Code Structure and Layout

## Code Configuration

## Setting up Microchip Studio for Bootloader Programming with J Link

## Setting up Arduino IDE

<!--


**NOTE:** Incomplete, more to come soon.



This project is based on the Arduino IDE workflow. Below steps outline steps necessary to install Arduino IDE and configure it for the SAMD microcontroller.



1. Install [Arduino IDE](https://www.arduino.cc/en/software) from [here](https://www.arduino.cc/en/software).

2. [Download the Arduino SAMD core](https://docs.arduino.cc/learn/starting-guide/cores/).

3. Download necessary libraries from library manager:



a. ArduinoLowPower

c. Sparkfun GNSS v3



5. To following needs to be downloaded directly from GitHub:

a. TinyBME280



4. Open Tiny4FSK.ino by double-clicking it (should open Arduino IDE).



The SAMD goes to sleep to save power. To achieve proper sleep, some edits to the SAMD core are necessary. To find the wiring.c file on your computer, [follow this guide](https:support.arduino.cc/hc/en-us/articles/4415103213714-Find-sketches-libraries-board-cores-and-other-files-on-your-computer).

Once there, comment out or completely delete this line as shown:



#ifndef VERY_LOW_POWER

// Setup all pins (digital and analog) in INPUT mode (default is nothing)

for (uint32_t ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )

{

pinMode( ul, INPUT ) ;

}

#endif



**NOTE: ** Arduino gives you an official warning:



> This breaks Arduino APIs since all pins are considered INPUT at startup. However, it really lowers the power consumption by a factor of 20 in low power mode (0.03mA vs 0.6mA).

>

Though from testing, there do not seem to be any performance issues.
-->

# Glider

## Required Parts for the Physical Glider

## Printing the Parts

## Making the Wing

## Making the Fuselage

## Making the Control Surfaces

## Assembling Everything

# Usage

## Power-on Cycle

## How to Actually Use in a Flight