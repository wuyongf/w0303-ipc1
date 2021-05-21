**Overall Functions**

1. two main functions: 
   
    (1) DoSchedules()

    (2) WaitSchedules()

**Todo:** 
1. Upload the whole program to Google Drive
2. Upload Communication Diagram (TCP/IP)
3. Upload Wiring Diagram (Electric)
4. For IPC1 program, the "Get/Post/Put Method" will cout the result. Need to fine tune.

***
2021/03/16

HA Onsite Setting

1. handle_01, handle_02, handle_03, handle_04.

2. handrail_01
         
         handrail_01_a
         handrail_01_b
   
3. protection_panel_01
      
         protection_panel_01_a
         protection_panel_01_b

4. handrail_02
   
         handrail_02_a
         handrail_02_b1
         handrail_02_b2
         handrail_02_b3
         handrail_02_c
   
5. protection_panel_02

         protection_panel_02_a
         protection_panel_02_b1
         protection_panel_02_b2
         protection_panel_02_b3
         protection_panel_02_c

6. foot_pedal_01

7. foot_pedal_02

8. basin_01

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

