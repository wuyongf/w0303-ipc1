# W0303 Overview

**This program aims to:**

1. Establish a control system which can control mobile manipulator to handle cleaning & disinfection tasks.
2. Interact with Web-based GUI via database(MSSQL).
3. Perform Point Cloud Algorithms to adjust localization error caused by ugv(mir100).

**The control system consists of several modules, including:**

1. Schedule Module: system entry module, interacting with other modules.
2. Arm Module (Motion Control/Status)
3. Ugv Module (Motion Control/Status)
4. Database Module (Detailed Functions Implementation)
5. Vision Module (Point Cloud/2d Image Process/Status)
6. Algorithms Module

##Get Start

1. Language: C++
2. Compiler: Cmake
3. CPU Architecture: x64
4. Package Manager: Vcpkg
5. Libraries: opencv4/nanodbc/glog/libmodbus/poco/eigen3/boost/pcl/openni2

## Usage

**Overall Functions**

1. two main functions:

   (1) DoSchedules()

   (2) WaitSchedules()


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




***
**Robortic Arm: TM-Flow Program**



***



## Contributing


## License