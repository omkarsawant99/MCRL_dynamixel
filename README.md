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
![image](https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/1346d1cd-f56d-4e3c-8642-b94c4f1f5e8a)

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



https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/d265cef0-e7b2-48eb-ba89-f562b1290b09

