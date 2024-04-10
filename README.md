

# StratoSoar MK2 - The Low-Power Autonomous High-Altitude Glider

**NOTE** - Docs are **NOT FINISHED** yet.

**WORK IN PROGRESS** - This project is still in the works. I don't recommend using it yet because it has the potential of being dangerous.

**NEW** - Working on making a real glider. 

**STATE as of 4/9/24** - Second draft of PCBs working!

<br>

<img src="https://github.com/crnicholson/StratoSoar-MK2/blob/main/Media/logo.png" alt="Logo" width="300"/><br>_StratoSoar Logo._

<img src="https://github.com/crnicholson/StratoSoar-MK2/blob/main/Media/boardPhotoMK2.0.png" alt="PCB" width="300"/><br>_StratoSoar MK2.0 Circuit Board._

## What Is the StratoSoar Project?

StratoSoar is a low-power, adaptive UAV, dropped from a weather balloon at approximately 80,000 feet in altitude. The main goal of StratoSoar is to select GPS coordinates for the glider to fly to, and then the glider will try to go there in an efficient manner, or intelligently choose a closer, more realistic landing site without any human interaction. Currently, we want StratoSoar to run off one AAA battery (or possibly two AAAs) for a 6 hour flight.

StratoSoar is a UAV. Here's a quick definition. UAVs, or Unmanned Aerial Vehicles, are autonomous aircraft, usually in the form of planes or drones. UAVs have many uses, like [delivering medicine, food, and supplies](https://www.flyzipline.com/) to remote areas. They also have a large presence in aerial combat with the military and aerial imaging. Some of the common types of UAVs include the common quadcopter (what you think of when you think of drones) and fixed wing aircraft (like StratoSoar). UAVs are usually quite large (5-15 meter wingspan, heavy (2-20 kg), and expensive ($100,000-10,000,000 USD). StratoSoar aims to combat the limitations of conventional UAVs with less than 250 grams of weight, 80 cm wingspan, and less than $150 (USD) in cost.

## What Are the Uses of StratoSoar?

Currently, the two main ideas we are thinking of include an educational kit and something for the study of aerodynamics of StratoSoar (of which very little exist). Other ideas include reusable radiosondes, remote medicine/parcel/etc. delivery, and atmospheric monitoring. 

## A Quick Note on MK1, Naming, and Revisions

StratoSoar MK1 was the precursor and the spark to StratoSoar MK2. StratoSoar MK1 was officially developed from October 1st to July 30th, although StratoSoar MK2 was unofficially being worked on in the months of March through June. StratoSoar MK1 failed to fly autonomously, and it also failed to release from the balloon. Major upgrades have been made since then.

StratoSoar MK2 officially started on October 1st, and has not ended yet. It will end sometime in July, when I launch StratoSoar from a weather balloon. A new major revision (eg. MK3) comes out every time when I release the glider from the balloon in the Summer. After the Summer, work on the next revision begins, with many updates and new features. 

A new sub-revision for lack of better word comes out every time I make a new PCB. For example, my first version of my PCB was MK2.0. At the time of writing, the current version of StratoSoar is MK2.1. A new version of 3D model is denoted by the Rev number. Rev starts at 0. 0 and 1 are not shown in this repo because they didn't turn out very good. The physical gliders are denoted by the 3D model number. Sometimes a small change is made with glider so the name of the glider is Rev x.x, for example. Sometimes the same happens with the 3D model, again it would be x.x. To put this in practice, the final glider may be MK2.5 Rev 7.3.1. The code has no version history in the way the 3D models and the PCB do.

One last thing. When I refer to StratoSoar, I am referring to the project as a whole, but more specifically, the most recent version. 

## What Includes the StratoSoar Project? What Are the Different Parts of it?

StratoSoar has many different parts, and they all combine to make this project. 

- **Code** - All of the code directory, and the programs needed to run it. These include the main `.ino`s and the accompanying settings files, along with the test files for the EEPROM.
- **Glider** - This is the real thing that flies! We include guides to make this, but it is really designed to be more free style, in a sense, where you can design your own glider and use StratoSoar's electronics.
- **PCB** - The circuit board is the basis of this project, it interfaces the code with the glider. StratoSoar includes the circuit board files and the schematics and everything needed to build a PCB.
- **3D Files** - StratoSoar requires some 3D printed parts to function, but these files also serve as an interactive build guide. StratoSoar includes the 3D files needed to print parts for the glider and the 3D files to assemble the glider.
- **Media** - The images and logos of StratoSoar. 
- **Docs** - StratoSoar has docs to help you assemble and work with it. It is this stuff right here! There is a folder called `Docs` which also includes some interesting legal information about StratoSoar. 
- **Cutdown Mechanism** - This is the thing that helps you release the glider from the balloon payload - or whatever you want to drop it from. This includes all of the above things but will be in a separate GitHub repository. 
- **Breadboard Ammeter** - This is an ammeter built into the breadboard to find extremely accurate current readings over time. There will be a small GitHub repo with the code and the wiring diagram. 

## How Does StratoSoar Work at a High Level?

StratoSoar is attached to a weather balloon via the cutdown mechanism. Once the balloon reaches the desired altitude, the cutdown mechanism drops the glider. The glider then descends, steering its way to specified coordinates. 

StratoSoar takes pride in being highly configurable and hackable, so there are many ways you change how it comes down built into the code. 

StratoSoar uses data from an IMU that goes through a Mahony filter to receive accurate AHRS, or Attitude Heading Reference System. This can tell StratoSoar the pitch, yaw, and roll of the glider. Then, StratoSoar integrates a GPS module to get the GPS coordinates. Both the GPS and the AHRS are used to compute the azimuth to find the turning angle of the glider. The turning angle tells the glider how much it needs to turn to be facing the correct yaw that will get the glider to the target location. Additionally, an algorithm is used to get the distance between the glider location's and the target location. 

With the turning angle computed, a PID mechanism is used to control the servo attached to the rudder of the glider. This PID mechanism learn from it's mistakes and can accurately aim the glider in the correct direction. 

The glider will fly to the selected coordinates, and depending on the user configuration, will either spiral down or release a parachute above the coordinates once it has reached them to safely descend. Additionally, if the glider is too low in altitude, prior to the flight the user can enter closer coordinates as backup landing locations. 

**More is a WIP!**

## What Are Some of the Features of StratoSoar?

## What Are the Uses of StratoSoar?

## Can I Buy a Kit or a Complete Product?

Currently, StratoSoar is not available to purchase online as a kit or complete product. We have plans in the next few years to release a kit for schools, education centers, hobbyists and a complete product for industry at an affordable price. You are welcome to make one in the mean time!

## How Can I Make One? Are There Docs?

One of the goals of StratoSoar was to make an affordable and open source system for anyone to perform stratospheric research, so all the files are available for free. This means that you can construct StratoSoar from scratch. **A guide to do this will be available soon. It is still in the works, but you can check out `DOCS.md` under the `Docs` folder.** 

## Helpful Sources and Credits

## Cutdown Mechanism

## Funding

We currently have very limited funding to complete this project. We would love any outside support we could get to elevate (no pun intended) this project to the next level. Reach out to me if you have interest at:

- [newbsballoons@gmail.com](mailto:newbsballoons@gmail.com) (organization email, recommended).

- [charlienicholsonr@gmail.com](mailto:charlienicholsonr@gmail.com) (personal email).

## Special Thanks

This project would not be possible without the following individuals:

- Max Kendall (W0MXX) for working closely together with me.

- Mike Malis for helping with the design of the glider. 

- Brett Miwa for schematic review, PCB design, electrical theory, and boost converter circuitry design.

- Bob Phinney (K5TEC) for providing NEST and securing funding.

- Mark Jessop (VK5QI) for answering all of my questions.

- Mike Hojnowski (KD2EAT) for PCB design and hardware/software debugging.

- My parents!

And these entities:

- New England Sci-Tech (NEST) for providing a maker space to work, funding, and hosting NEWBS.

- New England Weather Balloon Society (NEWBS) for expertise and the opportunity

## Affiliate Programs and Projects

- **LittleLoRa** - This is my high altitude tracking system designed to play in with my glider. It is still in development. 
- **NEWBS** - Max and Seth Kendall of NEWBS, or New England Weather Balloon Society, are helping me launch my project from a balloon. They are dedicated to helping people in New England launch weather balloons. 
- **New England Sci-Tech** - Bob Phinney of NEST is hosting NEWBS. NEST is a great maker space. If you are in the Greater Boston Area, go check them out!
- **StratoScience** - This is the official class that I'm in helping me launch my project. This is a high altitude engineering class for teens. Check them out if your are in the Greater Boston area!

## Still Have Questions?

Do you have any questions? Are the docs incomplete and you want to ask anything? Do you just want to say hi (or how cool my project is!)?

Reach out to me at [charlienicholsonr@gmail.com](mailto:charlienicholsonr@gmail.com).

## License

Both the software and documentation are under the [GNU GPL v3 license](https://choosealicense.com/licenses/gpl-3.0/). The hardware and the 3D files are under the [CERN Open Hardware Licence Version 2 - Strongly Reciprocal](https://choosealicense.com/licenses/cern-ohl-s-2.0/). The media is under the [CC BY 4.0 DEED](https://creativecommons.org/licenses/by/4.0/).
