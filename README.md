# ELEC6910R
ELEC6910R Course Project

The project consists of the following tasks:
1. Build 2D grid map by using laserscan data and show it via rviz
2. Control the mobile robot in the simulation environment with keyboard (drive it to move)
3. Image Recognition and Localization. There are five images of different people in the environment
and we need to do the following:
(a) judge whether the target images occurred in current vision data
(b) if yes, estimate the location of target images
(c) add markers to the map in rviz which stands for the target images position
4. Visual Servoing. There is a slowly moving yellow (rgb:255,255,0) ball in the environment
and we need to write program (rosnode, in c/c++ or python) to control the mobile robot to
follow the ball
5. The room is divided into several areas, let the robot judge which area it locates
6. Write a launch file to roslaunch all of above programs at once
7. Additional work: face recognition, automatic self-exploration

Basic project requirement is fulfilled with additional work including face recognition and automatic self-exploration (see in task 3 and task 4).

The above tasks are implemented in Python on Ubuntu 14.04 with the following setup:
1. ROS Indigo
2. V-REP_PRO_EDU_V3.3.2
3. OpenCV 3
4. Hector_slam package
