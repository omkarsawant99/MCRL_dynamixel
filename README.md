# Robot Simulation framework
## Introduction
Welcome to Robot Simulation framework, a versatile robotics framework designed to simulate robots with Pinocchio-Meshcat. 
This repository can simulate any controller with any reference trajectory for any robot. You can mix-and-match any controller, trajectory and robot.
After simulating, you can also deploy the controller directly onto real hardware with Dynamixel motors.

## Features
### Controllers
This folder should house controllers that you want to test. Currently, it has a script for impedance controller.

### Planners
This folders should have reference trajectories that you want the robot to follow. Current planners include -

**Adaptive planner:** This planner increases velocity as the end effector moves away from the start position, maintains constant velocity in the direction of the target and 
reduces velocity as the target is close.
It also replans trajectory at each step making it resistant to disturbances.

### Robot Models

This folder should have robot models defined in URDF

## Dependencies
The repo has the following dependencies

Numpy - https://numpy.org/install/

Scipy - https://scipy.org/install/

Pinocchio - https://stack-of-tasks.github.io/pinocchio/download.html

Meshcat-Python - https://github.com/meshcat-dev/meshcat-python

## Simulation
The visualization with disturbance should look like this. It is equally applied to each joint. The disturbance in this case is exaggerated for representation of the functionality. 

https://github.com/omkarsawant99/MCRL_dynamixel/assets/112906388/d265cef0-e7b2-48eb-ba89-f562b1290b09

## Real time operation
The script `mimic.py` is can be used to test the feedback from the motors. The script should work as follows -

The `main.py` script can be launched with the function `simulate_robot_real_time` to finally deploy the controller on the real robot. Please make sure the parameters of your controller (gain and damping) are set appropriately. The script should work as follows -
