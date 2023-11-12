# Robot Simulation framework
## Introduction
Welcome to Robot Simulation framework, a versatile robotics framework designed to simulate robots with Pinocchio-Meshcat. 
This repository can simulate any controller with any reference trajectory for any robot. You can mix-and-match any controller, trajectory and robot.

## Features
### Controllers
This folder should house controllers that you want to test. Currently, it has a script for impedance controller.
### Planners
This folders should have reference trajectories that you want the robot to follow. Current planners include -
**Adaptive planner:** This planner increases velocity as the end effector moves away from the start position, maintains constant velocity in the direction of the target and 
reduces velocity as the target is close.
It also replans trajectory at each step making it resistant to disturbances.
![image1](https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/8fb3d0c6-6d40-4b1d-a700-c44d3ee57314)
### Robot Models
This folder should have robot models defined in URDF

## Dependencies
The repo has the following dependencies

Numpy - https://numpy.org/install/

Scipy - https://scipy.org/install/

Pinocchio - https://stack-of-tasks.github.io/pinocchio/download.html

Meshcat-Python - https://github.com/meshcat-dev/meshcat-python

## Visualization
The visualization should look like this - 
https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/120f47b7-b841-477c-8876-71e5885d7625