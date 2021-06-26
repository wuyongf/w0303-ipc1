# W0303 Overview

This program establishes communication among robotic arm(tm5-900), ugv(mir100) and database(sql server).

## Overall Components

1. Schedule Module: Interact with Database & Arm & Ugv Module
2. Arm Module (Motion/Action/Status)
3. Ugv Module (Motion/Mission/Status)
4. Database Module (detailed functions implementation)
**5. Vision Module (Image Process/Status)**

## Usage




**Overall Functions**

1. two main functions: 
   
    (1) DoSchedules()

    (2) WaitSchedules()

**Todo:** 
1. Upload the whole program to Google Drive
2. Upload Communication Diagram (TCP/IP)
3. Upload Wiring Diagram (Electric)
4. For IPC1 program, the "Get/Post/Put Method" will cout the result. Need to fine tune.

[comment]: <> (details template)
<details open>
<summary>Todo List</summary>

1. Fine tune the program.
2. Sort out an overall workflow
3. workflow for vision_rs module.

</details>

[comment]: <> (draw a line)
***

<details open>
<summary>Template</summary>

Python >= 3.6.0 required with all [requirements.txt](https://github.com/ultralytics/yolov5/blob/master/requirements.txt) dependencies installed:
<!-- $ sudo apt update && apt install -y libgl1-mesa-glx libsm6 libxext6 libxrender-dev -->
```bash
$ git clone https://github.com/ultralytics/yolov5
$ cd yolov5
$ pip install -r requirements.txt
```

```bash
$ testing
```

</details>



2021/03/15

1. (done) for mir: how to create a mission based on database.

2. (done) for arm: need to assign a mission in sequence.

         workflow

         (1) preparation
   
            a.   move to <vision_init_positon>
            b.  execute vision job. 
            c. update ref-points.
   
         (2) execute the mission


***
**TM-Flow Related**

1. Force Control Node

   (1) tool

   (2) motion path

***
**CLion Configuration**

1. vcpkg

   (1) Search:
   
         .\vcpkg\vcpkg search [package name]

   (2) Install: for x64

         .\vcpkg\vcpkg install [package name]:x64-windows
   
   (3) Package list

         .\vcpkg\vcpkg install glog:x64-windows

         .\vcpkg\vcpkg install nanodbc:x64-windows

         libmodbus

         poco

         eigen3

         boost

## Contributing


## License