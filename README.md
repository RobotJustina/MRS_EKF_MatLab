# Learning tools for the localization of mobile robots using visual landmarks in the environment and the extended Kalman Filter

## Overview 
This proposal addresses the creation of educational tools that will help students with the concept of the location of a mobile robot using the extended Kalman filter (EKF) with landmarks placed on robot´s environment.


The localization of a robot is a fundamental tool for its navigation. There are localization methods based on different sources of information and hardware components, as there is no method that works properly in any scenario and for any robot. From the educational point of view, few students students are able to understand all the mathematical background involved in the localization process.



## Requirements
* Matlab 2020a (Required toolbox: Navigation Toolbox)
* MathWorks Student Competitions Team (2021). Mobile Robotics Simulation Toolbox (https://github.com/mathworks-robotics/mobile-robotics-simulation-toolbox), GitHub. Retrieved June 29, 2021.
* AJ Johnson (2021). error_ellipse (https://www.mathworks.com/matlabcentral/fileexchange/4705-error_ellipse), MATLAB Central File Exchange. Retrieved October 31, 2021. 
* ROS Melodic (http://wiki.ros.org/melodic/Installation/Ubuntu)


## Lessons:

Be sure you already have the requirements Check Lessons folder to start, the lessons are divided into 6 chapters:

* Chapter 1 Basics:
	It Covers all the basics to run simulations and understand the basic kinematics for a differential mobile robot.
<img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/Ch1.png" alt="Full System" width="440"/>

* Chapter 2 Sensors:
	At this chapter you will learn how to implement sensors such as Odometry, LIDAR, Object detection and algorithms like VFH for obstacle avoidance.
 <img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/Ch2.png" alt="Full System" width="440"/>
 
* Chapter 3 Path planning:
At this lessons you will learn how to plan a path and how to make the robot follows it (path tracking) using RRT* algorithm.
 <img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/Ch3.png" alt="Full System" width="440"/>
 
* Chapter 4 Summary of previous Chapters:
 In this lesson you will learn how to put all together and reduce your code from previous lessons.
 
* Chapter 5 Localization: 
In this chapter you will learn why is necessary a localization system and then you will implement the Extended Kalman Filter understanding the theory of each step. You can find a detailed explanation in the file Lecture_Kalman_filter_robots.pdf in this repository.

 <img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/Ch5.png" alt="Full System" width="440"/>
 
* Chapter 6 ROS Localization:
In this lesson We show you how   a localization system works along with MATLAB and ROS. And you will learn how to use the correct EKF parameters  using a ROSBAG.
 <img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/Ch6.png" alt="Full System" width="440"/>



* You can practice with different algorithms, maps (maps folder) and changing parameters to practice in different environments and situations.
 
<img src="https://raw.githubusercontent.com/RobotJustina/MRS_EKF_MatLab/main/img/EKF_office.PNG" alt="Full System" width="440"/>

Odom:Pink line

EKF pose prediction: Green line

Robot pose: Blue line

