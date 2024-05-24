# Experimental Implementation of Skeleton Tracking for Collision Avoidance in Collaborative Robotics

In this repository are uploaded all the codes to perform an human obstacle avoidance with the Universal Robot 5e manipulator. The human pose detection is performed with 3 Intelrealsense D455 RGB-D cameras using a tool based on Machine Learining technique,
Mediapipe, as skeleton detection system.

The project is composed of three main agents:
- Skeleton Detection performed by a standard PC (PC 1);
- Obstacle avoidance algorithm performed by another standard PC (PC 2);
- Universal Robot 5e with his programm used to receive commands from PC 2.

Among the three components there is TCP/IP connection, if you set the following IP adresses to the agents you can use the programm directly without changing adresses inside the codes:
- PC 1 ->169.254.0.20
- PC 2 ->169.254.0.25
- Robot UR5e ->169.254.0.4

In the following repo you will find three main folder, one for each agent:
1. Human_pose_detection: code to run on PC 1 to perform the pose detection and send obstacles human joint to the avoidance algorithm (PC 2)
2. Avoidance_UR5e: code to run on PC 2 to controll the robot and perform the avoidance receiving obstacles in real time from PC 1
3. UR_script: UR script code to run on the controller of the robot to enable the external controll by MatLab.

Step to run the system:
1. Calibrate the three DepthCameras obtaining the extrinsic and intrinsic parameters for each of them;
2. Run the Pose detection system with the file main_human_pose_detection.py
3. Run .....py that is inside the Avoidance Folder, this file read joints angle from the robot.
4. Run the Avoidance algorithm with the MatLab file..... .mat
5. Run the UR program on the UR techpendant.

Human Pose Detection software used:
1. Python 3.7.11;
2. Matlab Engine for Python (MatLab R2021b), check compatabilities option between Python and Matlab version on https://it.mathworks.com/support/requirements/python-compatibility.html.
   To intall the Engine follow https://it.mathworks.com/help/matlab/matlab_external/install-the-matlab-engine-for-python.html. All the function implemented on Matlab can be redone on Python avoiding to install the engine. Is an user choice.
4. Requirements list

In the following image is proposed the architecture of the overall system.

<img src="/Images/achitecture.png" alt="architecture" width="800"/>

## Human Pose Detection
In the folder there are three .py files, the main one to run is main_human_pose_detection.py. Inside this file are called the other files .py and the two matlab functions. In the function rototrasla_robot.m are used the extrinsic parameters of the cameras calibration saved in a Matlab variable T1r_opt.mat and so on.

## Avoidance Algorithm
In the folder is present the main file to run ..... and the .....py to read the joint angles.

## UR Script 
In the folder is present the script file that has to be calles from a .urp Universal Robot programm file inside the robot controller. Check the IP of the robot, has to be the same set in the codes file.


