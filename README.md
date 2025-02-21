# Séance
Semi-Autonomous Holonomic Battlebot

## Overview
This project aims to introduce more autonomy to the combat robotics scene. By combining some limited on-robot compute and sensing with a more powerful, stationary base station, we can start playing around with some cool new abilities and control schemes that would be difficult for a human driver to achieve precisely. The base station would track the position of both my robot and the opponent's, and would use this information to inform a variety of semi-autonomous behaviors.

The benefits become especially appealing when combined with a holonomic drive system (i.e. one where planar rotations and translations are decoupled). Examples of potentially useful abilities include:
- World-frame driving
  - All driver velocity inputs are with respect to the world frame, so the robot can be driven like a top-down video game
- Opponent rotational tracking
  - The robot automatically faces the opponent, keeping the weapon system pointed at them while the two robots move around the arena. This enables safer and more precise maneuvers, and makes it very hard for the opponent to get around to the more vulnerable sides and back of the bot
- Attack / evade macros
  - The robot can use a set of control macros for more precise attacks and evasions based on the relative position of the opponent and the arena walls

This code will focus primarily on the first two benefits for now, though the third one (and more) should be possible with this system.

Full autonomy is theoretically possible in the future, but for now I intend to keep the driver in-the-loop and use the extra processing / sensing to augment their control interface. Skillful driving is a huge components of combat robotics, and I don't think autonomous systems will supercede those human skills any time soon, especially when considering how chaotic robot fights are. Right now, the goal is to provide a technical edge that lets the driver more easily execute precise maneuvers that would be extremely difficult or impossible for a normal robot to do. However, in combat robotics, if something can go wrong, it usually does. As such, the driver will be expected to learn to pilot Séance skillfully and competitively without the driver assist functionalities as well.

This project will be implemented on small-scale combat robots initially, using either a 3lb "Beetleweight" or a 12lb "Hobbyweight." The holonomic drive will be achieved using either an X-drive or a swerve drive. The first prototypes of this system used an X-drive, but the relatively low traction was problematic in combat.

## The Code
This repository is divided into two main sections:
- Robot_Code
  - Runs on a microcontroller inside the robot, interfacing with the RC receiver, motor controllers, and sensors
- Base_Station_Code
  - Runs on a base station computer, interfacing with the RC transmitter, driver input device (currently an XBox controller), and remote sensing technology (most likely computer vision, possibly with fiducial assistance)

Check out the README file within each folder for more details on each section of the project.

## Attributions
Written and developed by Matt Nolan

For me, this idea originally started as an Applied Project toward the completion of my M.S in Robotics and Autonomous Systems. It functioned well as a student robotics project, but not fast or robust enough to be a competitive Battlebot. This repository aims to push it the rest of the way there.

That said, this idea is definitely not mine alone. I've spoken to several builders that are looking into similar-ish systems, and I hope to see autonomy become more popular as the sport evolves. Major props to the [Hacksmith](https://www.hacksmith.com/) team for building the first autonomous heavyweight (250lbs) Battlebot, [Orbitron](https://youtube.com/playlist?list=PLbncXbXlaNQemX6I22vQD6zWfIzIN9eOb&si=GFCeFmdy6aITD4MJ). Their videos started coming out about 9 months after I finished my Master's project, and played a large part in inspiring me to take this up again.

This project would be significantly more difficult without the hard work of several other people working on more general-purpose libraries. In particular, I want to highlight:
- [CRSFforArduino](https://github.com/ZZ-Cat/CRSFforArduino) by [ZZ-Cat](https://github.com/ZZ-Cat)
- [pico-dshot-bidir](https://github.com/josephduchesne/pico-dshot-bidir) by [josephduchesne](https://github.com/josephduchesne)

Their repositories make the robot itself much more polished and capable.