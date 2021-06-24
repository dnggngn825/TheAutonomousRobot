# The Autonomous Warehouse Robot

This is the source code for the project as a part of the **[ELEN90090 - Autonomous System Clinic](https://handbook.unimelb.edu.au/subjects/elen90090)** subject at the University of Melbourne. This project is written in C++ and Python as multiple [ROS](https://www.ros.org/) (Robotics Operation System) nodes, developed to run on the NVIDIA Jetson Xavier. The repository currently also includes code which has been run on the NVIDIA Jetson Xavier, and using desktop or laptop Ubuntu installations.

The template of the repo of this project has been provided by **Dr. Paul Beuchat** (who is also the subject coordinator) via [Gitlab](https://gitlab.unimelb.edu.au/asclinic/asclinic-system)

## About
### Objective
Design software pipeline for the robot to perform the task in a small-scale warehouse. Basically, the robot is required to maneuver to a dispenser for picking up items and to a receptacle for droping off.

### Project Overview
This project focuses on control and software design, based on the given robot platform. The robotic prototype will then be applied to work in a warehouse of no obstacles. However, there are two NOACCESS zones in the layout of warehouse. This robot contacts with the collection areas and receptacle areas via the markers, which helps to determine the object categories. Once the robot approaches these areas and a requested signal was sent, the items are manually dispensed.

### Contributor
- Hai Dang Nguyen (hain4@student.unimelb.edu.au)
- Hoang Viet Pham (hoangp2@student.unimelb.edu.au)
- Quang Trung Le (quangtrungl@student.unimelb.edu.au)

## Get Started
The following instructions detail the building and testing of a simple task cycle of the robot.

At the end of these instructions, you should be to compile and run the application, verifying that all development tools have been installed correctly.

### Installation Instructions

#### ROS Installation
The link below provides the instruction on how to install ROS on Linux or Unbuntu (for Windows desktop).

ROS Noetic Installation Guide: http://wiki.ros.org/noetic/Installation

### How to get the project
On the host, clone the project from git repository:
```bash
$ git clone https://github.com/dnggngn825/TheAutonomousRobot.git
```

### How to compile it
Then access to ```catkin_ws``` folder on Terminal using
```bash
$ cd TheAutonomousRobot/asclinic-system/catkin_ws
```
Then, on the same Terminal, we will compile the ```asclinic_pkg``` using
```bash
$ catkin_make
```
_Note: The command ```catkin_make``` needs to run to re-compile the package whenever there is any changes to the C++ files._

### How to run it
Before running it, we need to add several environment variables that ROS needs in order to work. On the same Terminal that we run ```catkin_make```, enter
```bash
$ source devel/setup.bash
```
Then we need to run ```roscore``` by
```bash
$ roscore
```
and let it run on a dedicated Terminal.

Now, open two Terminals and follow the same directory path to add several environment variables for ROS to work
```bash
$ cd TheAutonomousRobot/asclinic-system/catkin_ws
$ source devel/setup.bash
```
On Terminal 1, we launch the robot to start and wait for user command. By running the ```2nd_square_path_test_v2.launch``` file, all the nodes are on and waiting for message to be sent through. These nodes are in charge of different functionalities of the robot such as **motion planning**, **trajectory tracking**, **motor control**, **camera scan**, **sensing and localizing**.
```bash
$ roslaunch asclinic_pkg 2nd_square_path_test_v2.launch
```
We will leave Terminal 1 to run on itself, while we move to Terminal 2 and publish a message to ```/get_directory``` for the robot to start the task. The message type for this is ```pathCommand``` which is a customize ```.msg``` for presenting an array of 2-digit integer.
```bash
$ rostopic pub --once /get_directory pathCommand {"11"}
```
for picking up items from dispenser 1 and dropping off at receptacle 1 (one cycle)

or
```bash
$ rostopic pub --once /get_directory pathCommand {"21","11"}
```
with 2 requested cycle. The first one will be picking up items from dispenser 2 and dropping off at receptacle 1 (1st cycle). Then it will return to the start position before perfroming the next cycle, which is picking up items from dispenser 1 and dropping off at receptacle 1 (2nd cycle).

### Emergency Stop
For safety purpose, a dedicated topic ```/state_indicator``` was created to run on the back on ```motion_planning``` node for the user to interupt at any time with a single command line to switch the state of the robot. The idea behind this is to continuously publish a message to that topic and it will switch the state automatically within 3s.
```bash
$ rostopic pub /state_indicator std_msgs::String "stop"
```
