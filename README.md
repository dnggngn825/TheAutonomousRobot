# The Autonomous Warehouse Robot

This is the source code for the project as a part of the **[ELEN90090 - Autonomous System Clinic](https://handbook.unimelb.edu.au/subjects/elen90090)** subject at the University of Melbourne. This project is written in C++ and Python as multiple ROS (Robotics Operation System) nodes, developed to run on the NVIDIA Jetson Xavier. The repository currently also includes code which has been run on the NVIDIA Jetson Xavier, and using desktop or laptop Ubuntu installations.

## Objective
Design software pipeline for the robot to perform the task in a small-scale warehouse. Basically, the robot is required to maneuver to a dispenser for picking up items and to a receptacle for droping off.

### Project Overview
This project focuses on control and software design, based on the given robot platform. The robotic prototype will then be applied to work in a warehouse of no obstacles. However, there are two NOACCESS zones in the layout of warehouse. This robot contacts with the collection areas and receptacle areas via the markers, which helps to determine the object categories. Once the robot approaches these areas and a requested signal was sent, the items are manually dispensed.

## Get Started
The following instructions detail the building and testing of a simple task cycle of the robot.

At the end of these instructions, you should be to compile and run the application, verifying that all development tools have been installed correctly.

### Installation Instructions

#### Host Installation

#### ROS Installation

### How to get the project
On the host, clone the project from git repository:
```bash
$ git clone https://github.com/dnggngn825/TheAutonomousRobot.git
```
