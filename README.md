# Manipulator Simulation framework
## Introduction
Welcome to Robot Simulation framework, a versatile robotics framework designed to simulate robots with Pinocchio-Meshcat. 
This repository can simulate any controller with any reference trajectory for any robot. You can mix-and-match any controller, trajectory and robot.

## Features
### Controllers
This folder should house controllers that you want to test. Currently, it has a script for impedance controller.
MPC controller is still in progress.

### Planners
This folders should have reference trajectories that you want the robot to follow. Current planners include -

**Adaptive planner:** This planner increases velocity as the end effector moves away from the start position, maintains constant velocity in the direction of the target and 
reduces velocity as the target is close.
It also replans trajectory at each step making it resistant to disturbances.

### Robot Models

This folder should have robot models defined in URDF

### Utils
Here you can add functions to deploy your controller on a real robot. 
`dynamixel_utils.py` - This file contains functions to deploy controller on robot using dynamixel motors

## Dependencies
The repo has the following dependencies

Numpy - https://numpy.org/install/

Scipy - https://scipy.org/install/

Pinocchio - https://stack-of-tasks.github.io/pinocchio/download.html

Meshcat-Python - https://github.com/meshcat-dev/meshcat-python

## Visualization and Simulation
The visualization is done using Pinocchio-Meshcat environment. You can also introduce disturbances in the environment. 

### Disturbance in the end effector frame
This type of disturbance simulates a linear force applied to the end effector in the local frame. It can be activated by switching `disturbance_end_effector=True`. Below is the video of how the simulation should look like, with disturbance at 1.5 seconds -

https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/6dd150cd-9d53-4b3d-985b-351f1bf61015



[![Video](URL_of_Thumbnail_Image)](https://clipchamp.com/watch/XHLmpnnQldk/embed)


### Random disturbances at each joint
In this case, random disturbances is equally applied to each joint. It can be activated by switching `disturbance_all_joints=True`

