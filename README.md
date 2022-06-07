# VREP_MATLAB/SIMULINK
A simple demo for communicating between VREP (CoppeliaSim) and MATLAB/SIMULINK.

The demo is to control the virtual PANDA robot in VREP using MATLAB *.m code or only SIMULINK (via Sfunction).

# Platform
Matlab R2020a 64bit;

CoppeliaSim Edu, Version 4.1.0 (rev.1) 64bit.

# Contents
This demo folder includes, 

(1) Q_des, desired trajectory data in form of *.mat and *.txt, and a corresponding image;

(2) MATLAB *.m code for communication between MATLAB and VREP;

(3) SIMULINK *.slx file for communication between SIMULINK and VREP (via Sfunction);

(4) A scene *.ttt file is the virtual PANDA robot in VREP.

(5) File *v1.m can only send joint position command to VREP robot; file *v2.m can retrive joint position data and joint torque data from VREP robot.

![20210522_194541_PANDAinVREP](https://user-images.githubusercontent.com/34574771/119245433-98bbba80-bb36-11eb-8214-1a27b54c3b6c.png)


https://user-images.githubusercontent.com/34574771/119245920-42507b00-bb3a-11eb-81ab-96bdbdefb84c.mp4

