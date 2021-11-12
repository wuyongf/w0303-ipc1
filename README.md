# W0303 Overview

## Introduction

![RobotApperance](doc/resources/robot_apperance_1.jpg)

[WebGUI](https://robot.willsonic.com/) (not release to public. [Screenshot - 1](doc/resources/screenshot_publish_schedule.png) | [Screenshot - 2](doc/resources/screenshot_job_record.png)))

[Onsite Trial Video - 1](https://youtu.be/3_yQ5s5oSw4) | [Onsite Trial Video - 2](https://youtu.be/yljeNZbJ-PU)

**This program aims to:**

1. Establish a control system which can control mobile manipulator to handle cleaning & disinfection tasks.
2. Interact with Web-based GUI via database(MSSQL).
3. Perform Vision Algorithms(Point Cloud / QR Code) to adjust localization error caused by mobile robot.

**The control system consists of several modules, including:**

1. Schedule Module: system entry module, interacting with other modules.
2. Arm Module: Motion Control Module & Status Manager for Robotic Arm([tm5-900](https://www.tm-robot.com/en/regular-payload/)).
3. Ugv Module: Motion Control Module & Status Manager for Mobile Robot([mir100](https://www.mobile-industrial-robots.com/solutions/robots/mir100/)).
4. Database Module: Detailed Functions Implementation
5. Vision Module: Point Cloud/2d Image Processing Module
6. Algorithms Module: robotic arm TCP transformation

## Installation

**Prerequisite**

1. Language: C++
2. Compiler: Cmake
3. CPU Architecture: x64
4. Package Manager: Vcpkg
5. Libraries: opencv4/nanodbc/glog/libmodbus/poco/eigen3/boost/pcl/openni2

## Usage

## Contributing


## License