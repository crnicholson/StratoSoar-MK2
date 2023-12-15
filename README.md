### Note: This is a work in progress, and I warn you if you use my code or anything else here, stuff could go wrong. That being said, I hope you find some use out of this.

# StratoSoar MK2
StratoSoar is an autonomous glider dropped from a high-altitude balloon. This balloon is launched by NEWBS and NeSciTech, and my project is part of a larger project called StratoScience. StratoScience is a group of teens who work together to make interesting science and engineering projects that fly 80,000 feet up into the sky. 

# Goals
There are many goals for my project, but I'll outline some of the major ones:
1. To be able to fly in a uniform manner
2. To be able to steer the glider autonomously
3. To cleanly drop the glider from 75,000 feet
4. To keep the glider under 249g
5. To make the glider out of easily obtainable/cheap materials
6. To be able to collect environmental data from sensors and read that data
7. To make a super low-power flight controller based on Arduino
8. To be able to track the glider using LoRa and HAM radio

As far as potential uses, the list goes on and on:
1. Environmental monitoring
2. A reusable radiosonde
3. A reusable satellite (a motor and solar panel could be attached, leaving the glider flying around certain areas in times of emergency)
4. A moving mesh network node (for remote areas)
5. Aerial imagery (could be used to find sickened farmlands, etc.)
6. Small-scale deliveries in remote areas, such as blood or medicine
7. And of course, a spy drone

# Prerequisites
1. Understanding of Arduino and Arduino IDE
2. Understanding of electronics
3. Basic understanding of planes and gliders
4. Basic understanding of embedded systems/ICs
5. Knowledge of fabrication processes

# Using The Autopilot/Flight Controller
### NOTE: Currently, no GPS has been interfaced with the board. The board's location is static. This is getting worked on.
To use this repo in your own project, a few things have to be done. First, you have to either use my PCB or make your own based on the posted schematic. DO NOT think that you can switch out parts that I have used; catastrophic failure is imminent without changing the provided code. You HAVE to use an ATMega328P, running 3.3v at 8 mHz, or else the script won't work. That is just an example of a failure point; there are many more. I recommend building the circuit on a breadboard first. I won't outline the circuit too much seeing as there is a detailed schematic that should tell you everything you need. 

After you've purchased and designed a PCB, parts will need to get purchased, and don't be surprised if you're easily spending $150+ on components. You have to know what you're getting into. I've made the SMD pads on my board quite large to make it easily solderable, but you should be comfortable with SMD soldering. All the components you need are listed in the PCB and schematic files.

Once you have your board, it's time to program it. This can be a bit tricky, as there are two different things to program. There are two MCUs in my system. This is to enable low-power operation. The AHRS script that I'm using will go berserks in low power mode, but it already consumes minimal power. On the other hand, the GPS and the servo script are a power hog, so I've figured out ways to make it lower power. But, for my system to work properly, the two MCUs have to be sending data between each other. This is achieved by using SoftwareSerial, which sends yaw, pitch, temp, and pressure data. The sketch for the MCU that receives data is called `autopilot.vx.x.inn` and has a file, `settings.h` that you can change to make the system work differently. It is not recommended to change the code in the main .ino file. The MCU that sends data has a file named `autopilotIMU.vx.x` and doesn't have a settings file, but has a section where you can change values to your liking. This sketch can be a bit more confusing - you need to calibrate the IMU (the sensor that obtains position measurements) using an outdated Windows program. This process is described here by J. Remington: https://github.com/jremington/MPU-9250-AHRS. The actual programming is conducted in Arduino IDE. To do this, follow these steps:
1. Download this repo and the code.
2. Download the MPU9250 library and follow the calibration steps at https://github.com/jremington/MPU-9250-AHRS.
3. Download other libraries needed to make the code work.
4. Enter calibration values into `autopilotIMU.vx.x`.
5. Change the settings in both autopilots to your liking.
6. Select the right jumper for programming the right MCU on the PCB.
7. Connect the 3.3v FTDI breakout to the PCB headers, and plug in the other end into your computer.
9. Select the right serial port in Arduino IDE, and select "Arduino Pro Mini, 3.3v 8mHz" under the board options.
10. Program each board individually.
11. Disconnect the FTDI breakout and connect the peripherals.
12. Realize you've done something wrong and start getting mad.
13. Fix your problem.
14. Realize that this is a lot of steps to program a board.
15. Realize that is very complicated and I'm not explaining this well.
16. Be happy and be done!

This autopilot moves servos based on yaw and pitch values. It takes into account current GPS location, yaw, and target GPS location to calculate the azimuth, which with more math, can be turned into a turning angle. This turning angle is then pushed through a PID system that tells the servos where to move with even more math. This is the same for the pitch - it tries to keep the angle of attack at -10 degrees. MOSFETs are used in this program to turn the servos on and off to save power. The one I use is the 30N06L, a logic-level N-channel FET. I also use an NPN BJT (2N3906) to power on and off the PWM line. I do this so that when the FET is low, ground can't go through the servo PWM (signal) line, which would damage the servo and the MCU. The servos I use are quite special servos that are tiny and weigh 2 grams. A link to where I purchased them is included: **put link here**

The main MCU goes to sleep between servo movements to save power. Note that there are also no power LEDs to save further power. Input power ranges from 3.7v to 4.8v, with being on the higher side better. This means that a LiPo would be perfect for power input.

# Flying and Building the Glider
Okay, so you've spent too much time doing making and programming your flight controller, and you just want to fly your glider now. Understandable. Unfortunely, I don't really know how to build a glider myself, so I can't give you much advice. What I can tell you is that you should:
1. Buy a glider on Amazon.
2. Carve out a spot for your flight controller, battery, and servos.
3. Glue the servos to the glider and connect the push rods to the rudder and elevator.
4. Connect the servos to their respective ports on the PCB.
5. Plug in a LiPo to the PCB.
6. Chuck your glider and see what happens!

You can also experiment with building your own glider. There are plenty of kits and 3D printable plans, but you could go the hardcore way and build it from scratch by watching a YouTube video or reading a book.

# Analysing the Data
So, you've flown (and crashed) your glider, and you're eager to see what environmental data you've collected on your EEPROM. But, you can't take your EEPROM out, so you have to get creative. To do this, follow these steps:
1. Upload the program `externalEEPROMRead.ino` to the main MCU using the FTDI breakout board and Arduino IDE.
2. Pop open your serial monitor and see your results!

# Funding
We currently have very limited funding to complete this project. We would love any outside support we could get to elevate this project to the next level. Reach out to me if you have interest at:
1. charlienicholsonr@gmail.com (personal email).
2. newbsballoons@gmail.com (org email, recommended)

# Special Thanks and Credits
Thanks to the amazing team at NEWBS (New England Weather Balloon Society): Max and Seth Kendall for the opportunity. Thanks to Bob Phinney at NeSciTech for supporting NEWBS and providing a workspace. Thanks to Brett Miwa at NeSciTech for the electrical engineering advice. Thanks to J. Remington for the AHRS code. And last, but definitely not least, thanks to the large community of people on the Arduino Forum, Arduino Discord, StackExchange, and StackOverflow that indirectly contributed to this project.
