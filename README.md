

# StratoSoar MK2 - The Low-Power Autonomous High-Altitude Glider

**NOTE** - Docs are **NOT FINISHED** yet.

**WORK IN PROGRESS** - This project is still in the works. I don't recommend using it yet because it has the potential of being dangerous.

**NEW** - Working on reducing power consumption and physical glider.

**STATE as of 3/22/24** - Second draft of PCBs ordered and coming soon.

## Version History

A quick little summary of the versions and what worked and what didn't, plus what was changed between revisions.

- StratoSoar MK2.0 - First attempt, many issues. Wrong LDO, transistor, and TCXO footprint. Boost converter did not work. Reset button did not work. Main sketch works along with mostly everything else. Missing silkscreen designators and numerous cosmetic problems. GPS untested. IMU untested.
- StratoSoar MK2.1 - Not here yet!

<img src="https://github.com/crnicholson/StratoSoar-MK2/blob/main/Media/logo.png" alt="Logo" width="300"/><br>_StratoSoar Logo._

<img src="https://github.com/crnicholson/StratoSoar-MK2/blob/main/Media/boardPhotoMK2.0.png" alt="PCB" width="300"/><br>_StratoSoar MK2.0 Circuit Board._

## What Is the StratoSoar Project?

StratoSoar is a low-power, adaptive UAV, dropped from a weather balloon at approximately 80,000 feet in altitude. The main goal of StratoSoar is to select GPS co-ordinates for the glider to fly to, and then the glider will try to go there in an efficient manner, or intelligently choose a closer, more realistic landing site without any human interaction. Currently, we want StratoSoar to run off one AAA battery (or possibly two AAAs) for a 6 hour flight.

## What Are the Uses of StratoSoar?

Currently, the two main ideas we are thinking of include an educational kit and something for the study of aerodynamics of StratoSoar (of which very little exist). Other ideas include reusable radiosondes, remote medicine/parcel/etc. delivery, and atmospheric monitoring. 

## What Are Unmanned Aerial Vehicles?

UAVs, or Unmanned Aerial Vehicles, are autonomous aircraft, usually in the form of planes or drones. UAVs have many uses, like [delivering medicine, food, and supplies](https://www.flyzipline.com/) in remote areas or aerial imaging. They also have a large presence in aerial combat with the military. Some of the common types of UAVs include the common quadcopter (what you think of when you think of drones) and fixed wing aircraft (like StratoSoar). UAVs are usually quite large (5-15 meter wingspan, heavy (2-20 kg), and expensive ($100,000-10,000,000 USD). StratoSoar aims to combat the limitations of conventional UAVs with less than 250 grams of weight, 80 cm wingspan, and less than $150 (USD) in cost.

## What Includes the StratoSoar Project? What Are the Different Parts of it?

**NOTE:** Soon to be merged with following section.

StratoSoar has many parts, detailed below. When something is referred to StratoSoar, it is the collective interfacing of all of these parts that make it.

- **Code** - StratoSoar includes a codebase, which is found here. This is broken into two main parts - one bit of code for the main chip and one bit of code for the secondary chip. Additionally, there is some code for reading and writing to the onboard data storage. This peices of code are refered to as `autopilot.ino`, `autopilotIMU.ino`, and `externalEEPROMRead.ino`. There are also configuration files for all of the code.

- **PCB** - For StratoSoar to function, there needs to be an interface between the code and the physical glider. This is where the circuit board comes in, which houses all the electronics, the chips that hold the code, battery pack, and connections to the servos.

## StratoSoar Layout and Stack-Up

**NOTE:** Incomplete, more to come soon.

### Electronics

At the heart of StratoSoar, there is a main flight controller PCB. This is located in the front of the glider, contained in the bottom of a 3D printed case. The flight controller board also houses the battery. On the top of the 3D printed case, there is a GPS tracker to locate the glider during the flight and send real-time data. This GPS tracker is called LittleLoRa, which will be talked about more later.

### Wing and Control Surfaces

<!--
The glider has a wing, well, because it is a glider. This 5 mm thick foamboard wing measures 80 cm across, and at 10 cm from the wing tips, points up a bit to add a bit of dihedral for stability. The wing has a 40 cm long 2 mm OD carbon fiber rod inlay for added structural support. There are no control surfaces on the wings.

The 5 mm thick foam board rudder of the glider is attached with hot glue and friction to a 3D printed part, to the dimensions shown in the 3D model. The control surface of the rudder is a peice of foam of the same size. There is a 3D printed control horn inserted into the control surface foam, where there is a 3D printed pushrod interfacer attached to the control horn. The control surface of the rudder is attached to the main part of the rudder via packing tape, leaving a 3-5 mm gap between the two foam peices.

The elevator is similar to the rudder in the sense that it is also a 5 mm thick peice of foam board cut the length shown in the 3D model. The control surface of the elevator is once again the same size as the elevator, and both parts are attached through a peice of packing tape.
-->
### Plus More, Coming Soon!

## How Does StratoSoar Work?

<!--
**NOTE:** Incomplete, more to come soon.



StratoSoar uses data from
-->

## What Are Some of the Features?

## Parts and Materials

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

## Setting up Microchip Studio for Bootloader Programming

## Setting up Arduino IDE for USB Programming

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

## Code Configuration

<!--
**NOTE:** False information, an update will be coming soon.



User configuration of StratoSoar is **required**. The file to change the settings is located under `Code > Autopilots > autopilot > headers > settings.h`. Here are some instructions guiding you through configuration.



`STATUS_LED` - Comment out to disable verbose status LEDs on PCB.



`DEVMODE` - Comment out for flight mode. Disables Serial and enables deep sleep modes for lower power consumption.



**Everything below these values in the configuration file can go unchanged.** These are pin numbers, and unless you are making your own PCB, leave them be.



`EXTINT` - GPS EXTINT pin for longer packet delays.



`SUCCESS_LED` - Success LED pin.



`ERROR_LED` - Error LED pin.
-->

## PCB Configuration

<!--
**NOTE:** False information, an update will be coming soon.



The StratoSoar PCB has many configurable operating modes, pins and power sources. This section will outline these parts of the PCB.



The PCB contains two jumpers, JP1 and JP2.



**JP1** - Cut (desolder) to enable SW3 for power.



**JP2** - Control power source (USB and center for USB mode, center and BATT for battery mode).



There are also two switches/buttons, SW1 and SW2.



**SW1** - Reset button for the microcontroller.



**SW2** - Battery power switch. Connects battery output to boost converter input.
-->

## Can I Buy a Kit or a Complete Product?

Currently, StratoSoar is not available to purchase online as a kit or complete product. We have plans in the next few years to release a kit for schools, education centers, hobbyists and a complete product for industry at an affordable price. You are welcome to make one.

## How Can I Make One?

One of the goals of StratoSoar was to make an affordable and open source system for anyone to perform stratospheric research, so all the files are avilable for free. This means that you can construct StratoSoar from scratch. **A guide to do this will be available soon.**

If you don't want to wait, you can order all the parts for the PCB from the BOM, order the PCB from JLCPCB using the gerbers, and 3D print all the parts according to the 3D files. You would then have to order carbon fiber rods, and cut them to size. You would have to make a wing out of any material you want. Finally, you would have to be comfortable with SMD soldering according to the PCB files.

## Helpful Sources and Credits

## LittleLoRa

## Cutdown Mechanism

## Funding

We currently have very limited funding to complete this project. We would love any outside support we could get to elevate (no pun intended) this project to the next level. Reach out to me if you have interest at:

- [newbsballoons@gmail.com](mailto:newbsballoons@gmail.com) (organization email, recommended).

- [charlienicholsonr@gmail.com](mailto:charlienicholsonr@gmail.com) (personal email).

## Special Thanks

This project would not be possible without the following individuals:

- Max Kendall (W0MXX) for working closely together with me.

- Brett Miwa for schematic review, PCB design, electrical theory, and boost converter circuitry design.

- Bob Phinney (K5TEC) for providing NEST and securing funding.

- Mark Jessop (VK5QI) for answering all of my questions.

- Mike Hojnowski (KD2EAT) for PCB design and hardware/software debugging.

- My parents!

And these entities:

- New England Sci-Tech (NEST) for providing a makerspace to work, funding, and hosting NEWBS.

- New England Weather Balloon Society (NEWBS) for expertise and the opportunity.

And of course, I can't forget to thank all the individuals volunteering time developing such things as the Arduino core, Arduino libraries, KiCAD, and many other opensource tools.

Last but not least, thanks to all the people who helped indirectly on websites such as Stack Exchange, Stack Overflow, and NEWBS Discord server, to name a few.

## Affiliate Programs

## Sponsors

## Still Have Questions?

Do you have any questions? Are the docs incomplete and you want to ask anything? Do you just want to say hi?

Reach out to me at [charlienicholsonr@gmail.com](mailto:charlienicholsonr@gmail.com).

## License

Both the software and documentation are under the [GNU GPL v3 license](https://choosealicense.com/licenses/gpl-3.0/). The hardware and the 3D files are under the [CERN Open Hardware Licence Version 2 - Strongly Reciprocal](https://choosealicense.com/licenses/cern-ohl-s-2.0/). The media is under the [CC BY 4.0 DEED](https://creativecommons.org/licenses/by/4.0/).
